/*
 * Smoke + DHT11 + OLED + Buzzer (NO TERMINAL)
 * FRDM-K66F - BAREMETAL
 *
 * OLED:
 *   T:xxC
 *   H:yy%
 *   S:NRM / WRN / DNG
 *
 * BUZZER (continuous patterns):
 *  - DANGER (Smoke DNG or Temp >= 60C): continuous ON
 *  - WARNING (Smoke WRN or Temp>=40 or Hum>=80): repeating beeps (0/1/0/1 with clear OFF time)
 *  - Else: OFF
 *
 * DHT11 is read every 1000ms (safe for DHT11 ~1Hz).
 * DHT11 is double-read (use 2nd if OK, otherwise 1st) to reduce sticky readings.
 * Buzzer timing is non-blocking (state machine) and updated every 10ms.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"

#include "fsl_adc16.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_i2c.h"

/* ===================== MQ-2 ADC (PTB3) ===================== */
#define ADC_BASE           ADC0
#define ADC_GROUP          0U
#define ADC_CHANNEL        13U
#define ADC_VREF_MV        3300U
#define ADC_MAX_COUNTS     4095U

/* ===================== DHT11 (PTC3) ===================== */
#define DHT_PORT           PORTC
#define DHT_GPIO           GPIOC
#define DHT_PIN            3U

/* ===================== BUZZER (PTC4) ===================== */
#define BUZ_PORT           PORTC
#define BUZ_GPIO           GPIOC
#define BUZ_PIN            4U

/* ===================== Smoke thresholds (tune if needed) ===================== */
#define SMOKE_WARN_MV      1000U
#define SMOKE_DANGER_MV    1600U
#define SMOKE_HYST_MV      50U

/* ===================== Temp/Hum thresholds ===================== */
#define TEMP_WARN1_C       40U
#define TEMP_DANGER_C      60U
#define HUM_WARN1_PCT      80U

#define TEMP_HYST_C        1U
#define HUM_HYST_PCT       2U

/* ===================== Buzzer beep pattern (WRN) ===================== */
#define BEEP_ON_MS         80U
#define BEEP_OFF_MS        420U

/* ===================== DWT Timing ===================== */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

static void delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    uint32_t start  = DWT_GetCycles();
    while ((DWT_GetCycles() - start) < cycles) {}
}

static void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}

static uint32_t millis(void)
{
    uint32_t cpm = SystemCoreClock / 1000U;
    return DWT_GetCycles() / cpm;
}

/* ===================== DHT11 helpers ===================== */
static inline void dht_output(void) { DHT_GPIO->PDDR |=  (1U << DHT_PIN); }
static inline void dht_input(void)  { DHT_GPIO->PDDR &= ~(1U << DHT_PIN); }
static inline void dht_low(void)    { DHT_GPIO->PCOR  =  (1U << DHT_PIN); }
static inline void dht_high(void)   { DHT_GPIO->PSOR  =  (1U << DHT_PIN); }

static inline uint8_t dht_read_pin(void)
{
    return (uint8_t)((DHT_GPIO->PDIR >> DHT_PIN) & 1U);
}

static uint8_t wait_level(uint8_t level, uint32_t timeout_us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * timeout_us;
    uint32_t start  = DWT_GetCycles();

    while (dht_read_pin() != level)
    {
        if ((DWT_GetCycles() - start) > cycles)
            return 1;
    }
    return 0;
}

static uint8_t dht11_read(uint8_t *tempC, uint8_t *hum)
{
    uint8_t data[5] = {0};

    dht_output();
    dht_low();
    delay_ms(18);

    dht_high();
    delay_us(30);
    dht_input();

    if (wait_level(0, 120)) return 1;
    if (wait_level(1, 120)) return 2;
    if (wait_level(0, 120)) return 2;

    for (int i = 0; i < 40; i++)
    {
        if (wait_level(1, 120)) return 3;
        delay_us(35);
        if (dht_read_pin())
            data[i / 8] |= (1U << (7 - (i % 8)));
        if (wait_level(0, 150)) return 3;
    }

    uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (sum != data[4]) return 4;

    *hum   = data[0];
    *tempC = data[2];
    return 0;
}

/* ===================== Buzzer ===================== */
static void Buzzer_Set(int on)
{
    if (on) GPIO_PortSet(BUZ_GPIO, 1U << BUZ_PIN);
    else    GPIO_PortClear(BUZ_GPIO, 1U << BUZ_PIN);
}

typedef enum { BUZ_OFF=0, BUZ_WARN_BEEP=1, BUZ_DANGER_ON=2 } buz_mode_t;

typedef struct {
    buz_mode_t mode;
    uint32_t next_toggle_ms;
    uint8_t is_on;
} buzzer_sm_t;

static buzzer_sm_t g_buz = {BUZ_OFF, 0, 0};

static void Buzzer_SetMode(buz_mode_t m)
{
    if (g_buz.mode != m)
    {
        g_buz.mode = m;
        g_buz.is_on = 0;
        g_buz.next_toggle_ms = millis();
        Buzzer_Set(0);
    }
}

static void Buzzer_Tick(void)
{
    uint32_t now = millis();

    if (g_buz.mode == BUZ_OFF)
    {
        Buzzer_Set(0);
        g_buz.is_on = 0;
        return;
    }

    if (g_buz.mode == BUZ_DANGER_ON)
    {
        Buzzer_Set(1);
        g_buz.is_on = 1;
        return;
    }

    if (now >= g_buz.next_toggle_ms)
    {
        if (g_buz.is_on)
        {
            g_buz.is_on = 0;
            Buzzer_Set(0);
            g_buz.next_toggle_ms = now + BEEP_OFF_MS;
        }
        else
        {
            g_buz.is_on = 1;
            Buzzer_Set(1);
            g_buz.next_toggle_ms = now + BEEP_ON_MS;
        }
    }
}

/* ===================== ADC ===================== */
static void InitADC(void)
{
    adc16_config_t cfg;
    CLOCK_EnableClock(kCLOCK_Adc0);
    ADC16_GetDefaultConfig(&cfg);
    cfg.resolution = kADC16_ResolutionSE12Bit;
    ADC16_Init(ADC_BASE, &cfg);
    ADC16_EnableHardwareTrigger(ADC_BASE, false);
    ADC16_SetHardwareAverage(ADC_BASE, kADC16_HardwareAverageCount32);
    (void)ADC16_DoAutoCalibration(ADC_BASE);
}

static uint32_t ReadSmoke_mV(void)
{
    adc16_channel_config_t ch = {0};
    ch.channelNumber = ADC_CHANNEL;
    ch.enableInterruptOnConversionCompleted = false;

    ADC16_SetChannelConfig(ADC_BASE, ADC_GROUP, &ch);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC_BASE, ADC_GROUP))) {}

    uint32_t raw = ADC16_GetChannelConversionValue(ADC_BASE, ADC_GROUP);
    return (raw * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

/* ===================== GPIO ===================== */
static void InitGPIO(void)
{
    CLOCK_EnableClock(kCLOCK_PortC);

    gpio_pin_config_t out_cfg = {kGPIO_DigitalOutput, 0};

    PORT_SetPinMux(BUZ_PORT, BUZ_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(BUZ_GPIO, BUZ_PIN, &out_cfg);
    Buzzer_Set(0);

    PORT_SetPinMux(DHT_PORT, DHT_PIN, kPORT_MuxAsGpio);
    dht_input();
}

/* ===================== OLED SSD1306 I2C1 ===================== */
#define I2C_INST           I2C1
#define I2C_CLK_EN         kCLOCK_I2c1
#define I2C_BAUD           100000U

#define I2C1_SCL_PORT      PORTC
#define I2C1_SDA_PORT      PORTC
#define I2C1_SCL_PIN       10U
#define I2C1_SDA_PIN       11U

#define OLED_ADDR_3C       0x3C
#define OLED_ADDR_3D       0x3D

#define SSD1306_CTRL_CMD   0x00
#define SSD1306_CTRL_DATA  0x40

static uint8_t g_oled_addr  = OLED_ADDR_3C;
static uint8_t g_oled_ready = 0;

static status_t i2c_write_bytes(uint8_t addr7, const uint8_t *data, size_t len)
{
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = addr7;
    xfer.direction = kI2C_Write;
    xfer.data = (uint8_t *)data;
    xfer.dataSize = len;
    xfer.flags = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(I2C_INST, &xfer);
}

static bool i2c_ping(uint8_t addr7)
{
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = addr7;
    xfer.direction = kI2C_Write;
    xfer.data = NULL;
    xfer.dataSize = 0;
    xfer.flags = kI2C_TransferDefaultFlag;
    return (I2C_MasterTransferBlocking(I2C_INST, &xfer) == kStatus_Success);
}

static void ssd1306_cmd(uint8_t c)
{
    uint8_t buf[2] = { SSD1306_CTRL_CMD, c };
    (void)i2c_write_bytes(g_oled_addr, buf, sizeof(buf));
}

static void ssd1306_cmd_list(const uint8_t *cmds, size_t n)
{
    uint8_t buf[64];
    if (n + 1 > sizeof(buf)) return;
    buf[0] = SSD1306_CTRL_CMD;
    memcpy(&buf[1], cmds, n);
    (void)i2c_write_bytes(g_oled_addr, buf, n + 1);
}

static void ssd1306_data(const uint8_t *d, size_t n)
{
    uint8_t buf[17];
    while (n)
    {
        size_t chunk = (n > 16) ? 16 : n;
        buf[0] = SSD1306_CTRL_DATA;
        memcpy(&buf[1], d, chunk);
        (void)i2c_write_bytes(g_oled_addr, buf, chunk + 1);
        d += chunk;
        n -= chunk;
    }
}

static void ssd1306_set_cursor(uint8_t page, uint8_t col)
{
    ssd1306_cmd(0xB0 | (page & 0x07));
    ssd1306_cmd(0x00 | (col & 0x0F));
    ssd1306_cmd(0x10 | ((col >> 4) & 0x0F));
}

static void ssd1306_clear_page(uint8_t page)
{
    uint8_t zeros[128];
    memset(zeros, 0, sizeof(zeros));
    ssd1306_set_cursor(page, 0);
    ssd1306_data(zeros, 128);
}

static void ssd1306_init(void)
{
    const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, 0xA0, 0xC0, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6
    };
    ssd1306_cmd_list(init_seq, sizeof(init_seq));
    ssd1306_cmd(0xAF);
}

typedef struct { char ch; uint8_t col[5]; } glyph5_t;
static const glyph5_t font5x7[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {':', {0x00,0x36,0x36,0x00,0x00}},
    {'%', {0x62,0x64,0x08,0x13,0x23}},
    {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'0', {0x3E,0x41,0x41,0x41,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x62,0x51,0x49,0x49,0x46}},
    {'3', {0x22,0x41,0x49,0x49,0x36}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x2F,0x49,0x49,0x49,0x31}},
    {'6', {0x3E,0x49,0x49,0x49,0x32}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x26,0x49,0x49,0x49,0x3E}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'N', {0x7F,0x06,0x18,0x60,0x7F}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'W', {0x7F,0x20,0x18,0x20,0x7F}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'G', {0x3E,0x41,0x49,0x29,0x1E}},
};

static const uint8_t* glyph(char c)
{
    for (unsigned i = 0; i < (sizeof(font5x7)/sizeof(font5x7[0])); i++)
        if (font5x7[i].ch == c) return font5x7[i].col;
    return font5x7[0].col;
}

static void ssd1306_print(uint8_t page, uint8_t col, const char *s)
{
    ssd1306_set_cursor(page, col);
    while (*s)
    {
        const uint8_t *g = glyph(*s++);
        ssd1306_data(g, 5);
        uint8_t gap = 0x00;
        ssd1306_data(&gap, 1);
    }
}

static void OLED_Init(void)
{
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(I2C1_SCL_PORT, I2C1_SCL_PIN, kPORT_MuxAlt2);
    PORT_SetPinMux(I2C1_SDA_PORT, I2C1_SDA_PIN, kPORT_MuxAlt2);

    CLOCK_EnableClock(I2C_CLK_EN);

    i2c_master_config_t cfg;
    I2C_MasterGetDefaultConfig(&cfg);
    cfg.baudRate_Bps = I2C_BAUD;

    I2C_MasterInit(I2C_INST, &cfg, CLOCK_GetFreq(kCLOCK_BusClk));

    if (i2c_ping(OLED_ADDR_3C)) g_oled_addr = OLED_ADDR_3C;
    else if (i2c_ping(OLED_ADDR_3D)) g_oled_addr = OLED_ADDR_3D;
    else { g_oled_ready = 0; return; }

    ssd1306_init();
    ssd1306_clear_page(0);
    ssd1306_clear_page(2);
    ssd1306_clear_page(4);
    g_oled_ready = 1;
}

static void OLED_Show(uint8_t t, uint8_t h, bool dht_ok, const char *s_txt)
{
    if (!g_oled_ready) return;

    char line1[24];
    char line2[24];
    char line3[24];

    ssd1306_clear_page(0);
    ssd1306_clear_page(2);
    ssd1306_clear_page(4);

    if (dht_ok)
    {
        snprintf(line1, sizeof(line1), "T:%02uC            ", t);
        snprintf(line2, sizeof(line2), "H:%02u%%           ", h);
    }
    else
    {
        snprintf(line1, sizeof(line1), "T:--C            ");
        snprintf(line2, sizeof(line2), "H:--%%           ");
    }

    snprintf(line3, sizeof(line3), "S:%s             ", s_txt);

    ssd1306_print(0, 0, line1);
    ssd1306_print(2, 0, line2);
    ssd1306_print(4, 0, line3);
}

/* ===================== Main ===================== */
int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    InitGPIO();
    InitADC();
    DWT_Init();
    OLED_Init();

    delay_ms(2000);

    uint16_t hum_f4 = 0;
    bool hum_init = false;

    enum { S_NRM=0, S_WRN=1, S_DNG=2 } smoke_level = S_NRM;

    bool temp_warn40 = false;
    bool hum_warn80  = false;

    uint8_t last_temp = 0;
    uint8_t last_hum  = 0;
    bool last_dht_ok  = false;

    uint32_t last_sample_ms = millis();

    for (;;)
    {
        Buzzer_Tick();

        uint32_t now = millis();

        if ((now - last_sample_ms) >= 1000U)
        {
            last_sample_ms = now;

            uint32_t smoke_mv = ReadSmoke_mV();

            if (smoke_level == S_NRM)
            {
                if (smoke_mv >= SMOKE_DANGER_MV) smoke_level = S_DNG;
                else if (smoke_mv >= SMOKE_WARN_MV) smoke_level = S_WRN;
            }
            else if (smoke_level == S_WRN)
            {
                if (smoke_mv >= SMOKE_DANGER_MV) smoke_level = S_DNG;
                else if (smoke_mv < (SMOKE_WARN_MV - SMOKE_HYST_MV)) smoke_level = S_NRM;
            }
            else
            {
                if (smoke_mv < (SMOKE_DANGER_MV - SMOKE_HYST_MV))
                {
                    if (smoke_mv >= SMOKE_WARN_MV) smoke_level = S_WRN;
                    else smoke_level = S_NRM;
                }
            }

            uint8_t temp1=0, hum1=0, temp2=0, hum2=0;
            bool ok = false;

            uint8_t e1 = dht11_read(&temp1, &hum1);
            delay_ms(60);
            uint8_t e2 = dht11_read(&temp2, &hum2);

            if (e2 == 0) { last_temp = temp2; last_hum = hum2; ok = true; }
            else if (e1 == 0) { last_temp = temp1; last_hum = hum1; ok = true; }
            else ok = false;

            last_dht_ok = ok;

            uint8_t hum_show = last_hum;

            if (ok)
            {
                if (!hum_init)
                {
                    hum_f4 = (uint16_t)last_hum * 4U;
                    hum_init = true;
                }
                else
                {
                    hum_f4 = (uint16_t)((3U * hum_f4 + (uint16_t)last_hum * 4U) / 4U);
                }
                hum_show = (uint8_t)(hum_f4 / 4U);

                if (!temp_warn40) { if (last_temp >= TEMP_WARN1_C) temp_warn40 = true; }
                else { if (last_temp <= (TEMP_WARN1_C - TEMP_HYST_C)) temp_warn40 = false; }

                if (!hum_warn80) { if (hum_show >= HUM_WARN1_PCT) hum_warn80 = true; }
                else { if (hum_show <= (HUM_WARN1_PCT - HUM_HYST_PCT)) hum_warn80 = false; }
            }
            else
            {
                temp_warn40 = false;
                hum_warn80  = false;
            }

            const char *s_txt = "NRM";
            if (smoke_level == S_DNG) s_txt = "DNG";
            else if (smoke_level == S_WRN) s_txt = "WRN";

            OLED_Show(last_temp, hum_show, last_dht_ok, s_txt);

            bool danger = false;
            bool warn   = false;

            if (smoke_level == S_DNG) danger = true;
            if (last_dht_ok && last_temp >= TEMP_DANGER_C) danger = true;

            if (!danger)
            {
                if (smoke_level == S_WRN) warn = true;
                if (last_dht_ok && (temp_warn40 || hum_warn80)) warn = true;
            }

            if (danger)      Buzzer_SetMode(BUZ_DANGER_ON);
            else if (warn)   Buzzer_SetMode(BUZ_WARN_BEEP);
            else             Buzzer_SetMode(BUZ_OFF);
        }

        delay_ms(10);
    }
}
