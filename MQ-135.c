/*
 * main.c
 * ─────────────────────────────────────────────────────────────────────────────
 *  Sensors  : DHT11 (temp/hum), MQ-2 (smoke), MQ-135 (air quality),
 *             MLX90614 (IR temp), KY-026 (flame)
 *  Actuators: Buzzer, Water Pump, Fan
 *  Display  : SSD1306 OLED 128x64
 *  Comms    : ESP8266 via UART1 (JSON over serial)
 *
 *  Wiring summary
 *  ──────────────
 *  DHT11        OUT  →  PTC16
 *  MQ-2         AO   →  PTB4  (A3, ADC1 ch10)
 *  MQ-135       AO   →  PTB5  (A2, ADC1 ch11)
 *  KY-026       DO   →  PTB19 (D9)
 *  Buzzer            →  PTC8
 *  Pump              →  PTD2
 *  Fan               →  PTD3
 *  MLX90614     SCL  →  PTB2  (A5 / G12, I2C0)
 *               SDA  →  PTB3  (A4 / G11, I2C0)
 *  OLED SSD1306 SCL  →  PTC10 (I2C1)
 *               SDA  →  PTC11 (I2C1)
 *  ESP8266      TX   →  PTC3  (UART1 RX)
 *               RX   →  PTC4  (UART1 TX)
 *  Debug LED         →  PTC17
 * ─────────────────────────────────────────────────────────────────────────────
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
#include "fsl_uart.h"

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Pin / peripheral definitions                                               */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* Debug LED */
#define LED_PORT            PORTC
#define LED_GPIO            GPIOC
#define LED_PIN             17U

/* ADC1 – MQ-2 and MQ-135 */
#define ADC_BASE            ADC1
#define ADC_GROUP           0U
#define MQ2_CH              10U       /* PTB4 = A3 */
#define MQ135_CH            11U       /* PTB5 = A2 */
#define ADC_VREF_MV         3300U
#define ADC_MAX_COUNTS      4095U

/* DHT11 */
#define DHT_PORT            PORTC
#define DHT_GPIO            GPIOC
#define DHT_PIN             16U

/* Buzzer */
#define BUZ_PORT            PORTC
#define BUZ_GPIO            GPIOC
#define BUZ_PIN             8U

/* Water pump */
#define PUMP_PORT           PORTD
#define PUMP_GPIO           GPIOD
#define PUMP_PIN            2U

/* Fan */
#define FAN_PORT            PORTD
#define FAN_GPIO            GPIOD
#define FAN_PIN             3U

/* KY-026 flame sensor */
#define FLAME_PORT          PORTB
#define FLAME_GPIO          GPIOB
#define FLAME_PIN           19U

/* UART1 – ESP8266 */
#define WIFI_UART           UART1
#define WIFI_UART_CLK       kCLOCK_Uart1
#define WIFI_UART_PORT      PORTC
#define WIFI_UART_TX_PIN    4U
#define WIFI_UART_RX_PIN    3U
#define WIFI_UART_BAUD      9600U

/* I2C0 – MLX90614 (PTB2 = SCL, PTB3 = SDA) */
#define MLX_I2C             I2C0
#define MLX_I2C_CLK         kCLOCK_I2c0
#define MLX_SCL_PORT        PORTB
#define MLX_SDA_PORT        PORTB
#define MLX_SCL_PIN         2U        /* A5 / G12 */
#define MLX_SDA_PIN         3U        /* A4 / G11 */
#define MLX_ADDR            0x5A
#define MLX_REG_TOBJ1       0x07

/* I2C1 – OLED SSD1306 (PTC10 = SCL, PTC11 = SDA) */
#define OLED_I2C            I2C1
#define OLED_I2C_CLK        kCLOCK_I2c1
#define OLED_SCL_PORT       PORTC
#define OLED_SDA_PORT       PORTC
#define OLED_SCL_PIN        10U
#define OLED_SDA_PIN        11U
#define OLED_ADDR_3C        0x3C
#define OLED_ADDR_3D        0x3D
#define OLED_CTRL_CMD       0x00
#define OLED_CTRL_DATA      0x40

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Thresholds                                                                 */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* MQ-2 smoke */
#define SMOKE_WARN_MV       1600U
#define SMOKE_DANGER_MV     2000U
#define SMOKE_HYST_MV       50U

/* MQ-135 air quality */
#define AQ_GOOD_MV          800U
#define AQ_FAIR_MV          1500U
#define AQ_POOR_MV          2500U

/* DHT11 temperature / humidity */
#define TEMP_WARN_C         40U
#define TEMP_DANGER_C       60U
#define TEMP_HYST_C         1U
#define HUM_WARN_PCT        80U
#define HUM_HYST_PCT        2U

/* Pump and fan */
#define PUMP_ON_TEMP_C      26U
#define PUMP_OFF_TEMP_C     25U
#define FAN_ON_TEMP_C       27U
#define FAN_OFF_TEMP_C      26U

/* Buzzer beep pattern */
#define BEEP_ON_MS          80U
#define BEEP_OFF_MS         420U

/* Startup delays */
#define INIT_DELAY_MS       500U
#define I2C_SETTLE_MS       200U

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  DWT cycle-counter timing                                                   */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t DWT_Cycles(void) { return DWT->CYCCNT; }

static void delay_us(uint32_t us)
{
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    uint32_t start = DWT_Cycles();
    while ((DWT_Cycles() - start) < ticks) {}
}

static void delay_ms(uint32_t ms) { while (ms--) delay_us(1000U); }

static uint32_t millis(void)
    { return DWT_Cycles() / (SystemCoreClock / 1000U); }

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Debug LED                                                                  */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void LED_Init(void)
{
    CLOCK_EnableClock(kCLOCK_PortC);
    gpio_pin_config_t cfg = {kGPIO_DigitalOutput, 0};
    PORT_SetPinMux(LED_PORT, LED_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(LED_GPIO, LED_PIN, &cfg);
    GPIO_PortClear(LED_GPIO, 1U << LED_PIN);
}

static void LED_On(void)     { GPIO_PortSet(LED_GPIO,    1U << LED_PIN); }
static void LED_Off(void)    { GPIO_PortClear(LED_GPIO,  1U << LED_PIN); }
static void LED_Toggle(void) { GPIO_PortToggle(LED_GPIO, 1U << LED_PIN); }

static void LED_Blink(uint32_t times, uint32_t period_ms)
{
    for (uint32_t i = 0; i < times; i++) {
        LED_On();  delay_ms(period_ms);
        LED_Off(); delay_ms(period_ms);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  GPIO – buzzer, DHT, pump, fan, flame                                       */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void Buzzer_Set(bool on)
{
    if (on) GPIO_PortSet(BUZ_GPIO,   1U << BUZ_PIN);
    else    GPIO_PortClear(BUZ_GPIO, 1U << BUZ_PIN);
}

static void Pump_Set(bool on)
{
    if (on) GPIO_PortSet(PUMP_GPIO,   1U << PUMP_PIN);
    else    GPIO_PortClear(PUMP_GPIO, 1U << PUMP_PIN);
}

static void Fan_Set(bool on)
{
    if (on) GPIO_PortSet(FAN_GPIO,   1U << FAN_PIN);
    else    GPIO_PortClear(FAN_GPIO, 1U << FAN_PIN);
}

static bool Flame_Detected(void)
    { return ((FLAME_GPIO->PDIR >> FLAME_PIN) & 1U) == 1U; }

static void InitGPIO(void)
{
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);

    gpio_pin_config_t out = {kGPIO_DigitalOutput, 0};
    gpio_pin_config_t in  = {kGPIO_DigitalInput,  0};

    /* Buzzer */
    PORT_SetPinMux(BUZ_PORT,   BUZ_PIN,   kPORT_MuxAsGpio);
    GPIO_PinInit(BUZ_GPIO,     BUZ_PIN,   &out); Buzzer_Set(false);

    /* DHT11 – starts as input */
    PORT_SetPinMux(DHT_PORT, DHT_PIN, kPORT_MuxAsGpio);
    DHT_GPIO->PDDR &= ~(1U << DHT_PIN);

    /* Pump */
    PORT_SetPinMux(PUMP_PORT,  PUMP_PIN,  kPORT_MuxAsGpio);
    GPIO_PinInit(PUMP_GPIO,    PUMP_PIN,  &out); Pump_Set(false);

    /* Fan */
    PORT_SetPinMux(FAN_PORT,   FAN_PIN,   kPORT_MuxAsGpio);
    GPIO_PinInit(FAN_GPIO,     FAN_PIN,   &out); Fan_Set(false);

    /* Flame sensor */
    PORT_SetPinMux(FLAME_PORT, FLAME_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(FLAME_GPIO,   FLAME_PIN, &in);

    delay_ms(50);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Buzzer state machine                                                       */
/* ═══════════════════════════════════════════════════════════════════════════ */

typedef enum { BUZ_OFF = 0, BUZ_BEEP = 1, BUZ_SOLID = 2 } buz_mode_t;

static struct {
    buz_mode_t mode;
    uint32_t   next_ms;
    bool       on;
} g_buz = {BUZ_OFF, 0, false};

static void Buzzer_SetMode(buz_mode_t m)
{
    if (g_buz.mode == m) return;
    g_buz.mode    = m;
    g_buz.on      = false;
    g_buz.next_ms = millis();
    Buzzer_Set(false);
}

static void Buzzer_Tick(void)
{
    uint32_t now = millis();
    switch (g_buz.mode) {
        case BUZ_OFF:
            Buzzer_Set(false);
            break;
        case BUZ_SOLID:
            Buzzer_Set(true);
            break;
        case BUZ_BEEP:
            if (now >= g_buz.next_ms) {
                g_buz.on = !g_buz.on;
                Buzzer_Set(g_buz.on);
                g_buz.next_ms = now + (g_buz.on ? BEEP_ON_MS : BEEP_OFF_MS);
            }
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  ADC1 – MQ-2 and MQ-135                                                    */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void ADC_Init(void)
{
    adc16_config_t cfg;
    CLOCK_EnableClock(kCLOCK_Adc1);
    ADC16_GetDefaultConfig(&cfg);
    cfg.resolution = kADC16_ResolutionSE12Bit;
    ADC16_Init(ADC_BASE, &cfg);
    ADC16_EnableHardwareTrigger(ADC_BASE, false);
    ADC16_SetHardwareAverage(ADC_BASE, kADC16_HardwareAverageCount32);
    (void)ADC16_DoAutoCalibration(ADC_BASE);
}

static uint32_t ADC_ReadMV(uint32_t channel)
{
    adc16_channel_config_t ch = {0};
    ch.channelNumber                       = channel;
    ch.enableInterruptOnConversionCompleted = false;
    ADC16_SetChannelConfig(ADC_BASE, ADC_GROUP, &ch);
    while (!(ADC16_GetChannelStatusFlags(ADC_BASE, ADC_GROUP)
             & kADC16_ChannelConversionDoneFlag)) {}
    return (ADC16_GetChannelConversionValue(ADC_BASE, ADC_GROUP)
            * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

static const char* AQ_Label(uint32_t mv)
{
    if (mv < AQ_GOOD_MV) return "GOOD";
    if (mv < AQ_FAIR_MV) return "FAIR";
    if (mv < AQ_POOR_MV) return "POOR";
    return "BAD";
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  DHT11                                                                      */
/* ═══════════════════════════════════════════════════════════════════════════ */

static inline void dht_dir_out(void) { DHT_GPIO->PDDR |=  (1U << DHT_PIN); }
static inline void dht_dir_in(void)  { DHT_GPIO->PDDR &= ~(1U << DHT_PIN); }
static inline void dht_low(void)     { DHT_GPIO->PCOR  =  (1U << DHT_PIN); }
static inline void dht_high(void)    { DHT_GPIO->PSOR  =  (1U << DHT_PIN); }
static inline uint8_t dht_pin(void)
    { return (uint8_t)((DHT_GPIO->PDIR >> DHT_PIN) & 1U); }

static bool dht_wait(uint8_t level, uint32_t timeout_us)
{
    uint32_t ticks = (SystemCoreClock / 1000000U) * timeout_us;
    uint32_t start = DWT_Cycles();
    while (dht_pin() != level)
        if ((DWT_Cycles() - start) > ticks) return false;
    return true;
}

/* Returns true on success, fills *tempC and *hum */
static bool DHT11_Read(uint8_t *tempC, uint8_t *hum)
{
    uint8_t data[5] = {0};

    /* Start signal */
    dht_dir_out(); dht_low(); delay_ms(18);
    dht_high(); delay_us(30); dht_dir_in();

    /* Sensor response */
    if (!dht_wait(0, 120)) return false;
    if (!dht_wait(1, 120)) return false;
    if (!dht_wait(0, 120)) return false;

    /* 40 data bits */
    for (int i = 0; i < 40; i++) {
        if (!dht_wait(1, 120)) return false;
        delay_us(35);
        if (dht_pin()) data[i / 8] |= (1U << (7 - (i % 8)));
        if (!dht_wait(0, 150)) return false;
    }

    /* Checksum */
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
        return false;

    *hum   = data[0];
    *tempC = data[2];
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  I2C helpers – shared pin config                                            */
/* ═══════════════════════════════════════════════════════════════════════════ */

static const port_pin_config_t k_i2c_pin_cfg = {
    kPORT_PullUp,
    kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable,
    kPORT_OpenDrainEnable,
    kPORT_LowDriveStrength,
    kPORT_MuxAlt2,
    kPORT_UnlockRegister
};

static status_t I2C_Write(I2C_Type *base, uint8_t addr,
                           const uint8_t *data, size_t len)
{
    i2c_master_transfer_t x;
    memset(&x, 0, sizeof(x));
    x.slaveAddress = addr;
    x.direction    = kI2C_Write;
    x.data         = (uint8_t *)data;
    x.dataSize     = len;
    x.flags        = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(base, &x);
}

static status_t I2C_ReadReg(I2C_Type *base, uint8_t addr, uint8_t reg,
                             uint8_t *rxbuf, size_t rxlen)
{
    i2c_master_transfer_t x;
    memset(&x, 0, sizeof(x));
    x.slaveAddress   = addr;
    x.direction      = kI2C_Read;
    x.subaddress     = reg;
    x.subaddressSize = 1;
    x.data           = rxbuf;
    x.dataSize       = rxlen;
    x.flags          = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(base, &x);
}

static bool I2C_Ping(I2C_Type *base, uint8_t addr)
{
    i2c_master_transfer_t x;
    memset(&x, 0, sizeof(x));
    x.slaveAddress = addr;
    x.direction    = kI2C_Write;
    x.data         = NULL;
    x.dataSize     = 0;
    x.flags        = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(base, &x) == kStatus_Success;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  MLX90614 – I2C0 (PTB2 SCL, PTB3 SDA)                                     */
/* ═══════════════════════════════════════════════════════════════════════════ */

static bool g_mlx_ok = false;

static void MLX90614_Init(void)
{
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinConfig(MLX_SCL_PORT, MLX_SCL_PIN, &k_i2c_pin_cfg);
    PORT_SetPinConfig(MLX_SDA_PORT, MLX_SDA_PIN, &k_i2c_pin_cfg);

    CLOCK_EnableClock(MLX_I2C_CLK);
    i2c_master_config_t cfg;
    I2C_MasterGetDefaultConfig(&cfg);
    cfg.baudRate_Bps = 100000U;
    I2C_MasterInit(MLX_I2C, &cfg, CLOCK_GetFreq(kCLOCK_BusClk));
    delay_ms(100);

    g_mlx_ok = I2C_Ping(MLX_I2C, MLX_ADDR);
    if (g_mlx_ok) LED_Blink(4, 50);
}

/* Returns temperature in tenths of °C, or INT16_MIN on error */
static int16_t MLX90614_ReadTemp(void)
{
    if (!g_mlx_ok) return INT16_MIN;
    uint8_t buf[3] = {0};
    if (I2C_ReadReg(MLX_I2C, MLX_ADDR, MLX_REG_TOBJ1, buf, 3) != kStatus_Success)
        return INT16_MIN;
    uint16_t raw = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    return (int16_t)(((int32_t)raw * 2 - 27315) / 10);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  OLED SSD1306 – I2C1 (PTC10 SCL, PTC11 SDA)                               */
/* ═══════════════════════════════════════════════════════════════════════════ */

static uint8_t g_oled_addr  = OLED_ADDR_3C;
static bool    g_oled_ok    = false;

/* Low-level OLED I2C wrappers */
static void oled_cmd(uint8_t c)
{
    uint8_t b[2] = {OLED_CTRL_CMD, c};
    I2C_Write(OLED_I2C, g_oled_addr, b, 2);
}

static void oled_cmd_list(const uint8_t *cmds, size_t n)
{
    uint8_t b[64];
    if (n + 1 > sizeof(b)) return;
    b[0] = OLED_CTRL_CMD;
    memcpy(&b[1], cmds, n);
    I2C_Write(OLED_I2C, g_oled_addr, b, n + 1);
}

static void oled_data(const uint8_t *d, size_t n)
{
    uint8_t b[17];
    while (n) {
        size_t chunk = (n > 16) ? 16 : n;
        b[0] = OLED_CTRL_DATA;
        memcpy(&b[1], d, chunk);
        I2C_Write(OLED_I2C, g_oled_addr, b, chunk + 1);
        d += chunk; n -= chunk;
    }
}

static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_cmd(0xB0 | (page & 0x07));
    oled_cmd(0x00 | (col & 0x0F));
    oled_cmd(0x10 | ((col >> 4) & 0x0F));
}

static void oled_clear_page(uint8_t page)
{
    uint8_t z[128]; memset(z, 0, 128);
    oled_set_cursor(page, 0); oled_data(z, 128);
}

static void oled_hw_init(void)
{
    const uint8_t seq[] = {
        0xAE,               /* display off              */
        0xD5, 0x80,         /* clock divide             */
        0xA8, 0x3F,         /* multiplex ratio          */
        0xD3, 0x00,         /* display offset           */
        0x40,               /* start line               */
        0x8D, 0x14,         /* charge pump on           */
        0x20, 0x02,         /* page addressing mode     */
        0xA0,               /* segment remap normal     */
        0xC0,               /* COM scan normal          */
        0xDA, 0x12,         /* COM pins                 */
        0x81, 0xCF,         /* contrast                 */
        0xD9, 0xF1,         /* pre-charge               */
        0xDB, 0x40,         /* VCOMH deselect           */
        0xA4,               /* entire display on (RAM)  */
        0xA6,               /* normal (non-inverted)    */
        0xAF                /* display on               */
    };
    oled_cmd_list(seq, sizeof(seq));
}

/* 5×7 font – uppercase + digits + symbols */
typedef struct { char ch; uint8_t col[5]; } glyph_t;
static const glyph_t k_font[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}}, {':', {0x00,0x36,0x36,0x00,0x00}},
    {'.', {0x00,0x60,0x60,0x00,0x00}}, {'%', {0x62,0x64,0x08,0x13,0x23}},
    {'-', {0x08,0x08,0x08,0x08,0x08}}, {'+', {0x08,0x08,0x3E,0x08,0x08}},
    {'0', {0x3E,0x41,0x41,0x41,0x3E}}, {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x62,0x51,0x49,0x49,0x46}}, {'3', {0x22,0x41,0x49,0x49,0x36}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}}, {'5', {0x2F,0x49,0x49,0x49,0x31}},
    {'6', {0x3E,0x49,0x49,0x49,0x32}}, {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}}, {'9', {0x26,0x49,0x49,0x49,0x3E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}}, {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}}, {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}}, {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x29,0x1E}}, {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}}, {'K', {0x7F,0x08,0x14,0x22,0x41}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}}, {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x06,0x18,0x60,0x7F}}, {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}}, {'Q', {0x1C,0x22,0x26,0x22,0x1D}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}}, {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}}, {'U', {0x3F,0x40,0x40,0x40,0x3F}},
    {'W', {0x7F,0x20,0x18,0x20,0x7F}},
};

static const uint8_t* glyph_lookup(char c)
{
    for (unsigned i = 0; i < sizeof(k_font) / sizeof(k_font[0]); i++)
        if (k_font[i].ch == c) return k_font[i].col;
    return k_font[0].col; /* fallback: space */
}

static void oled_print(uint8_t page, uint8_t col, const char *s)
{
    oled_set_cursor(page, col);
    while (*s) {
        oled_data(glyph_lookup(*s++), 5);
        uint8_t gap = 0; oled_data(&gap, 1);
    }
}

static void OLED_Init(void)
{
    LED_On();
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(OLED_SCL_PORT, OLED_SCL_PIN, &k_i2c_pin_cfg);
    PORT_SetPinConfig(OLED_SDA_PORT, OLED_SDA_PIN, &k_i2c_pin_cfg);
    delay_ms(I2C_SETTLE_MS);

    CLOCK_EnableClock(OLED_I2C_CLK);
    delay_ms(10);

    const uint32_t bauds[] = {100000U, 50000U, 10000U};
    bool found = false;

    for (int attempt = 0; attempt < 3 && !found; attempt++) {
        i2c_master_config_t cfg;
        I2C_MasterGetDefaultConfig(&cfg);
        cfg.baudRate_Bps = bauds[attempt];
        I2C_MasterInit(OLED_I2C, &cfg, CLOCK_GetFreq(kCLOCK_BusClk));
        delay_ms(50);

        if      (I2C_Ping(OLED_I2C, OLED_ADDR_3C)) { g_oled_addr = OLED_ADDR_3C; found = true; LED_Blink(2, 100); }
        else if (I2C_Ping(OLED_I2C, OLED_ADDR_3D)) { g_oled_addr = OLED_ADDR_3D; found = true; LED_Blink(3, 100); }

        if (!found) {
            /* Bus recovery: clock out 9 SCL pulses */
            I2C_MasterDeinit(OLED_I2C);
            delay_ms(10);
            PORT_SetPinMux(OLED_SCL_PORT, OLED_SCL_PIN, kPORT_MuxAsGpio);
            gpio_pin_config_t scl_out = {kGPIO_DigitalOutput, 1};
            GPIO_PinInit(GPIOC, OLED_SCL_PIN, &scl_out);
            for (int p = 0; p < 9; p++) {
                GPIO_PortClear(GPIOC, 1U << OLED_SCL_PIN); delay_us(5);
                GPIO_PortSet(GPIOC,   1U << OLED_SCL_PIN); delay_us(5);
            }
            PORT_SetPinConfig(OLED_SCL_PORT, OLED_SCL_PIN, &k_i2c_pin_cfg);
            delay_ms(10);
        }
    }

    if (!found) { g_oled_ok = false; LED_Off(); return; }

    g_oled_ok = true;
    delay_ms(I2C_SETTLE_MS);
    oled_hw_init();
    delay_ms(100);

    for (uint8_t p = 0; p < 8; p++) oled_clear_page(p);
    oled_print(0, 0, "SYSTEM INIT OK");
    oled_print(2, 0, "READY");
    LED_Off();
}

static void OLED_Update(uint8_t temp, uint8_t hum, bool dht_ok,
                         const char *smoke, bool flame,
                         int16_t ir, const char *aq)
{
    if (!g_oled_ok) return;
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < 100U) return;
    last_ms = now;

    char ln[24];

    oled_clear_page(0);
    if (dht_ok) snprintf(ln, sizeof(ln), "T:%02uC H:%02u%%      ", temp, hum);
    else        snprintf(ln, sizeof(ln), "T:--C H:--%%      ");
    oled_print(0, 0, ln);

    oled_clear_page(2);
    if (ir != INT16_MIN) {
        int16_t iw = ir / 10, ifr = ir % 10;
        if (ifr < 0) ifr = -ifr;
        snprintf(ln, sizeof(ln), "IR:%d.%dC         ", iw, ifr);
    } else {
        snprintf(ln, sizeof(ln), "IR:--C           ");
    }
    oled_print(2, 0, ln);

    oled_clear_page(4);
    snprintf(ln, sizeof(ln), "S:%s F:%s       ", smoke, flame ? "DETECT" : "NONE");
    oled_print(4, 0, ln);

    oled_clear_page(6);
    snprintf(ln, sizeof(ln), "AQ:%s             ", aq);
    oled_print(6, 0, ln);

    LED_Toggle();
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  ESP8266 UART1 – JSON telemetry                                             */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void UART_Init_WiFi(void)
{
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(WIFI_UART_PORT, WIFI_UART_TX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(WIFI_UART_PORT, WIFI_UART_RX_PIN, kPORT_MuxAlt3);
    CLOCK_EnableClock(WIFI_UART_CLK);
    uart_config_t cfg;
    UART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = WIFI_UART_BAUD;
    cfg.enableTx     = true;
    cfg.enableRx     = true;
    UART_Init(WIFI_UART, &cfg, 180000000U);
    delay_ms(100);
}

static void UART_SendJSON(uint8_t temp, uint8_t hum,
                           uint8_t smoke_code, bool flame,
                           int16_t ir, const char *aq,
                           uint32_t uptime_s)
{
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
        "{\"T\":%u,\"H\":%u,\"S\":%u,\"F\":%u,\"IR\":%d,\"AQ\":\"%s\",\"ts\":%lu}\r\n",
        temp, hum, smoke_code, (unsigned)flame, (int)ir, aq,
        (unsigned long)uptime_s);
    if (n > 0)
        UART_WriteBlocking(WIFI_UART, (uint8_t *)buf, (size_t)n);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  main                                                                       */
/* ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    /* Core init */
    DWT_Init();
    delay_ms(INIT_DELAY_MS);
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    DWT_Init();

    /* Peripherals */
    LED_Init();
    LED_Blink(1, 500);

    InitGPIO();
    ADC_Init();

    delay_ms(INIT_DELAY_MS);
    OLED_Init();

    delay_ms(INIT_DELAY_MS);
    MLX90614_Init();

    delay_ms(INIT_DELAY_MS);
    UART_Init_WiFi();

    delay_ms(2000);
    LED_Blink(2, 200);

    /* ── Runtime state ───────────────────────────────────────────────────── */
    typedef enum { S_NRM = 0, S_WRN = 1, S_DNG = 2 } smoke_lvl_t;
    smoke_lvl_t smoke_lvl = S_NRM;

    uint8_t  last_temp   = 0;
    uint8_t  last_hum    = 0;
    bool     last_dht_ok = false;
    uint16_t hum_f4      = 0;
    bool     hum_init    = false;

    bool temp_warn = false;
    bool hum_warn  = false;
    bool pump_on   = false;
    bool fan_on    = false;
    bool flame     = false;

    int16_t    ir_temp   = INT16_MIN;
    const char *aq_label = "GOOD";

    uint32_t last_sample_ms = millis();
    uint32_t last_heartbeat = millis();
    uint32_t uptime_s       = 0;

    /* ── Main loop ───────────────────────────────────────────────────────── */
    for (;;)
    {
        Buzzer_Tick();

        uint32_t now = millis();

        /* Heartbeat blink every 2 s */
        if (now - last_heartbeat >= 2000U) {
            last_heartbeat = now;
            LED_Blink(1, 50);
        }

        /* 1 Hz sampling */
        if (now - last_sample_ms >= 1000U)
        {
            last_sample_ms = now;
            uptime_s++;

            /* ── Flame & IR ─────────────────────────────────────────── */
            flame   = Flame_Detected();
            ir_temp = MLX90614_ReadTemp();

            /* ── MQ-2 smoke (hysteresis state machine) ──────────────── */
            uint32_t smoke_mv = ADC_ReadMV(MQ2_CH);
            switch (smoke_lvl) {
                case S_NRM:
                    if      (smoke_mv >= SMOKE_DANGER_MV) smoke_lvl = S_DNG;
                    else if (smoke_mv >= SMOKE_WARN_MV)   smoke_lvl = S_WRN;
                    break;
                case S_WRN:
                    if      (smoke_mv >= SMOKE_DANGER_MV)                 smoke_lvl = S_DNG;
                    else if (smoke_mv <  SMOKE_WARN_MV - SMOKE_HYST_MV)  smoke_lvl = S_NRM;
                    break;
                case S_DNG:
                    if (smoke_mv < SMOKE_DANGER_MV - SMOKE_HYST_MV)
                        smoke_lvl = (smoke_mv >= SMOKE_WARN_MV) ? S_WRN : S_NRM;
                    break;
            }

            /* ── MQ-135 air quality ─────────────────────────────────── */
            aq_label = AQ_Label(ADC_ReadMV(MQ135_CH));

            /* ── DHT11 (two attempts) ───────────────────────────────── */
            uint8_t t1 = 0, h1 = 0, t2 = 0, h2 = 0;
            bool ok = false;
            bool r1 = DHT11_Read(&t1, &h1);
            delay_ms(60);
            bool r2 = DHT11_Read(&t2, &h2);
            if      (r2) { last_temp = t2; last_hum = h2; ok = true; }
            else if (r1) { last_temp = t1; last_hum = h1; ok = true; }
            last_dht_ok = ok;

            uint8_t hum_show = last_hum;
            if (ok) {
                /* EMA filter on humidity (α = 0.25) */
                if (!hum_init) { hum_f4 = (uint16_t)last_hum * 4; hum_init = true; }
                else            hum_f4  = (uint16_t)((3U * hum_f4 + (uint16_t)last_hum * 4U) / 4U);
                hum_show = (uint8_t)(hum_f4 / 4U);

                /* Warning flags with hysteresis */
                if (!temp_warn) { if (last_temp >= TEMP_WARN_C)              temp_warn = true; }
                else            { if (last_temp <= TEMP_WARN_C - TEMP_HYST_C) temp_warn = false; }
                if (!hum_warn)  { if (hum_show  >= HUM_WARN_PCT)             hum_warn  = true; }
                else            { if (hum_show  <= HUM_WARN_PCT - HUM_HYST_PCT) hum_warn = false; }

                /* Pump control */
                if (!pump_on) { if (last_temp >= PUMP_ON_TEMP_C || flame) { pump_on = true;  Pump_Set(true);  } }
                else          { if (last_temp <= PUMP_OFF_TEMP_C && !flame) { pump_on = false; Pump_Set(false); } }

                /* Fan control */
                if (!fan_on) { if (last_temp >= FAN_ON_TEMP_C)  { fan_on = true;  Fan_Set(true);  } }
                else         { if (last_temp <= FAN_OFF_TEMP_C) { fan_on = false; Fan_Set(false); } }

            } else {
                /* DHT failed – safe state */
                temp_warn = hum_warn = pump_on = fan_on = false;
                Pump_Set(false); Fan_Set(false);
            }

            /* ── OLED & UART ────────────────────────────────────────── */
            const char *smoke_txt = (smoke_lvl == S_DNG) ? "DNG"
                                  : (smoke_lvl == S_WRN) ? "WRN" : "NRM";

            OLED_Update(last_temp, hum_show, last_dht_ok,
                        smoke_txt, flame, ir_temp, aq_label);

            UART_SendJSON(last_temp, hum_show, (uint8_t)smoke_lvl,
                          flame, ir_temp, aq_label, uptime_s);

            /* ── Buzzer priority: danger > warn > off ───────────────── */
            bool danger = flame
                       || (smoke_lvl == S_DNG)
                       || (last_dht_ok && last_temp >= TEMP_DANGER_C);

            bool warn = !danger
                     && (smoke_lvl == S_WRN
                         || (last_dht_ok && (temp_warn || hum_warn)));

            if      (danger) Buzzer_SetMode(BUZ_SOLID);
            else if (warn)   Buzzer_SetMode(BUZ_BEEP);
            else             Buzzer_SetMode(BUZ_OFF);
        }

        delay_ms(10);
    }
}
