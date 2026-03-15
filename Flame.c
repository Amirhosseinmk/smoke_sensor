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

/* ===================== Debug LED (PTC17) ===================== */
#define DEBUG_LED_PORT     PORTC
#define DEBUG_LED_GPIO     GPIOC
#define DEBUG_LED_PIN      17U

/* ===================== MQ-2 ADC (PTB3) ===================== */
#define ADC_BASE           ADC0
#define ADC_GROUP          0U
#define ADC_CHANNEL        13U
#define ADC_VREF_MV        3300U
#define ADC_MAX_COUNTS     4095U

/* ===================== DHT11 (PTC16) ===================== */
#define DHT_PORT           PORTC
#define DHT_GPIO           GPIOC
#define DHT_PIN            16U

/* ===================== BUZZER (PTC8) ===================== */
#define BUZ_PORT           PORTC
#define BUZ_GPIO           GPIOC
#define BUZ_PIN            8U

/* ===================== WATER PUMP (PTD2) ===================== */
#define PUMP_PORT          PORTD
#define PUMP_GPIO          GPIOD
#define PUMP_PIN           2U

/* ===================== FAN (PTD3) ===================== */
#define FAN_PORT           PORTD
#define FAN_GPIO           GPIOD
#define FAN_PIN            3U

/* ===================== FLAME SENSOR KY-026 (PTB19 = D9) ===================== */
#define FLAME_PORT         PORTB
#define FLAME_GPIO         GPIOB
#define FLAME_PIN          19U

/* ===================== ESP8266 UART1 (PTC4=TX, PTC3=RX) ===================== */
#define WIFI_UART          UART1
#define WIFI_UART_CLK      kCLOCK_Uart1
#define WIFI_UART_PORT     PORTC
#define WIFI_UART_TX_PIN   4U
#define WIFI_UART_RX_PIN   3U
#define WIFI_UART_BAUD     9600U

/* ===================== MLX90614 I2C address ===================== */
#define MLX90614_ADDR      0x5A
#define MLX90614_RAM_TOBJ1 0x07  /* Object temperature register */

/* ===================== Smoke thresholds ===================== */
#define SMOKE_WARN_MV      1600U
#define SMOKE_DANGER_MV    2000U
#define SMOKE_HYST_MV      50U

/* ===================== Temp/Hum thresholds ===================== */
#define TEMP_WARN1_C       40U
#define TEMP_DANGER_C      60U
#define HUM_WARN1_PCT      80U
#define TEMP_HYST_C        1U
#define HUM_HYST_PCT       2U

/* ===================== Pump thresholds ===================== */
#define PUMP_ON_TEMP_C     26U
#define PUMP_OFF_TEMP_C    25U

/* ===================== Fan thresholds ===================== */
#define FAN_ON_TEMP_C      27U
#define FAN_OFF_TEMP_C     26U

/* ===================== Buzzer beep pattern ===================== */
#define BEEP_ON_MS         80U
#define BEEP_OFF_MS        420U

/* ===================== Delays ===================== */
#define WINDOWS_I2C_DELAY_MS    200
#define WINDOWS_INIT_DELAY_MS   500

/* ===================== DWT Timing ===================== */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t DWT_GetCycles(void) { return DWT->CYCCNT; }

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
    return DWT_GetCycles() / (SystemCoreClock / 1000U);
}

/* ===================== Debug LED ===================== */
static void DebugLED_Init(void)
{
    CLOCK_EnableClock(kCLOCK_PortC);
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};
    PORT_SetPinMux(DEBUG_LED_PORT, DEBUG_LED_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(DEBUG_LED_GPIO, DEBUG_LED_PIN, &led_config);
    GPIO_PortClear(DEBUG_LED_GPIO, 1U << DEBUG_LED_PIN);
}

static void DebugLED_On(void)     { GPIO_PortSet(DEBUG_LED_GPIO,    1U << DEBUG_LED_PIN); }
static void DebugLED_Off(void)    { GPIO_PortClear(DEBUG_LED_GPIO,  1U << DEBUG_LED_PIN); }
static void DebugLED_Toggle(void) { GPIO_PortToggle(DEBUG_LED_GPIO, 1U << DEBUG_LED_PIN); }

static void DebugLED_Blink(uint32_t times, uint32_t delay)
{
    for (uint32_t i = 0; i < times; i++) {
        DebugLED_On();  delay_ms(delay);
        DebugLED_Off(); delay_ms(delay);
    }
}

/* ===================== ESP8266 UART ===================== */
static void WIFI_UART_Init(void)
{
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(WIFI_UART_PORT, WIFI_UART_TX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(WIFI_UART_PORT, WIFI_UART_RX_PIN, kPORT_MuxAlt3);
    CLOCK_EnableClock(WIFI_UART_CLK);

    uart_config_t cfg;
    UART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = WIFI_UART_BAUD;
    cfg.enableTx = true;
    cfg.enableRx = true;
    UART_Init(WIFI_UART, &cfg, 180000000U);

    delay_ms(100);
}

static void WIFI_UART_SendLine(uint8_t t, uint8_t h, const char *s_txt,
                                bool flame, int16_t ir_temp)
{
    char buf[80];
    int n = snprintf(buf, sizeof(buf), "T=%u,H=%u,S=%s,F=%s,IR=%d\r\n",
                     t, h, s_txt, flame ? "DET" : "NO", ir_temp);
    if (n > 0)
        UART_WriteBlocking(WIFI_UART, (uint8_t*)buf, (size_t)n);
}

/* ===================== DHT11 ===================== */
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
        if ((DWT_GetCycles() - start) > cycles) return 1;
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

/* ===================== Flame sensor ===================== */
static bool Flame_Detected(void)
{
    return ((FLAME_GPIO->PDIR >> FLAME_PIN) & 1U) == 1U;
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
    if (g_buz.mode != m) {
        g_buz.mode = m;
        g_buz.is_on = 0;
        g_buz.next_toggle_ms = millis();
        Buzzer_Set(0);
    }
}

static void Buzzer_Tick(void)
{
    uint32_t now = millis();
    if (g_buz.mode == BUZ_OFF)       { Buzzer_Set(0); g_buz.is_on = 0; return; }
    if (g_buz.mode == BUZ_DANGER_ON) { Buzzer_Set(1); g_buz.is_on = 1; return; }
    if (now >= g_buz.next_toggle_ms) {
        if (g_buz.is_on) { g_buz.is_on = 0; Buzzer_Set(0); g_buz.next_toggle_ms = now + BEEP_OFF_MS; }
        else             { g_buz.is_on = 1; Buzzer_Set(1); g_buz.next_toggle_ms = now + BEEP_ON_MS;  }
    }
}

/* ===================== Pump ===================== */
static void Pump_Set(bool on)
{
    if (on) GPIO_PortSet(PUMP_GPIO, 1U << PUMP_PIN);
    else    GPIO_PortClear(PUMP_GPIO, 1U << PUMP_PIN);
}

/* ===================== Fan ===================== */
static void Fan_Set(bool on)
{
    if (on) GPIO_PortSet(FAN_GPIO, 1U << FAN_PIN);
    else    GPIO_PortClear(FAN_GPIO, 1U << FAN_PIN);
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
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);

    gpio_pin_config_t out_cfg = {kGPIO_DigitalOutput, 0};
    gpio_pin_config_t in_cfg  = {kGPIO_DigitalInput,  0};

    PORT_SetPinMux(BUZ_PORT, BUZ_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(BUZ_GPIO, BUZ_PIN, &out_cfg);
    Buzzer_Set(0);

    PORT_SetPinMux(DHT_PORT, DHT_PIN, kPORT_MuxAsGpio);
    dht_input();

    PORT_SetPinMux(PUMP_PORT, PUMP_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(PUMP_GPIO, PUMP_PIN, &out_cfg);
    Pump_Set(false);

    PORT_SetPinMux(FAN_PORT, FAN_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(FAN_GPIO, FAN_PIN, &out_cfg);
    Fan_Set(false);

    PORT_SetPinMux(FLAME_PORT, FLAME_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(FLAME_GPIO, FLAME_PIN, &in_cfg);

    delay_ms(50);
}

/* ===================== OLED + MLX90614 I2C ===================== */
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
    xfer.direction    = kI2C_Write;
    xfer.data         = (uint8_t *)data;
    xfer.dataSize     = len;
    xfer.flags        = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(I2C_INST, &xfer);
}

static status_t i2c_read_bytes(uint8_t addr7, uint8_t reg,
                                uint8_t *rxbuf, size_t rxlen)
{
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress   = addr7;
    xfer.direction      = kI2C_Read;
    xfer.subaddress     = reg;
    xfer.subaddressSize = 1;
    xfer.data           = rxbuf;
    xfer.dataSize       = rxlen;
    xfer.flags          = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(I2C_INST, &xfer);
}

static bool i2c_ping(uint8_t addr7)
{
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = addr7;
    xfer.direction    = kI2C_Write;
    xfer.data         = NULL;
    xfer.dataSize     = 0;
    xfer.flags        = kI2C_TransferDefaultFlag;
    return (I2C_MasterTransferBlocking(I2C_INST, &xfer) == kStatus_Success);
}

/* ===================== MLX90614 ===================== */
static bool g_mlx_ready = false;

static void MLX90614_Init(void)
{
    delay_ms(50);
    if (i2c_ping(MLX90614_ADDR)) {
        g_mlx_ready = true;
        DebugLED_Blink(4, 50);
    } else {
        g_mlx_ready = false;
    }
}

/* Returns object temperature in degrees Celsius x10 (e.g. 255 = 25.5C)
   Returns INT16_MIN on error */
static int16_t MLX90614_ReadObjTemp(void)
{
    if (!g_mlx_ready) return INT16_MIN;

    uint8_t buf[3] = {0};
    status_t status = i2c_read_bytes(MLX90614_ADDR, MLX90614_RAM_TOBJ1, buf, 3);
    if (status != kStatus_Success) return INT16_MIN;

    /* buf[0] = data low, buf[1] = data high, buf[2] = PEC */
    uint16_t raw = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);

    /* Convert from Kelvin x 50 to Celsius x10 */
    /* raw * 0.02 - 273.15 = Celsius */
    /* raw * 2 - 27315 = Celsius x 100 */
    int32_t tempC_x10 = ((int32_t)raw * 2 - 27315) / 10;

    return (int16_t)tempC_x10;
}

/* ===================== OLED SSD1306 ===================== */
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
    while (n) {
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
    {'.', {0x00,0x60,0x60,0x00,0x00}},
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
    {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'K', {0x7F,0x08,0x14,0x22,0x41}},
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
    while (*s) {
        const uint8_t *g = glyph(*s++);
        ssd1306_data(g, 5);
        uint8_t gap = 0x00;
        ssd1306_data(&gap, 1);
    }
}

static void I2C_ConfigurePins(void)
{
    CLOCK_EnableClock(kCLOCK_PortC);

    port_pin_config_t i2c_pin_config = {
        kPORT_PullUp,
        kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_OpenDrainEnable,
        kPORT_LowDriveStrength,
        kPORT_MuxAlt2,
        kPORT_UnlockRegister
    };

    PORT_SetPinConfig(I2C1_SCL_PORT, I2C1_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C1_SDA_PORT, I2C1_SDA_PIN, &i2c_pin_config);

    delay_ms(10);
}

static void OLED_Init(void)
{
    DebugLED_On();

    I2C_ConfigurePins();
    delay_ms(WINDOWS_I2C_DELAY_MS);

    CLOCK_EnableClock(I2C_CLK_EN);
    delay_ms(10);

    uint32_t baud_rates[] = {100000, 50000, 10000};
    bool found = false;

    for (int i = 0; i < 3 && !found; i++) {
        i2c_master_config_t cfg;
        I2C_MasterGetDefaultConfig(&cfg);
        cfg.baudRate_Bps = baud_rates[i];
        I2C_MasterInit(I2C_INST, &cfg, CLOCK_GetFreq(kCLOCK_BusClk));
        delay_ms(50);

        if (i2c_ping(OLED_ADDR_3C))      { g_oled_addr = OLED_ADDR_3C; found = true; DebugLED_Blink(2, 100); }
        else if (i2c_ping(OLED_ADDR_3D)) { g_oled_addr = OLED_ADDR_3D; found = true; DebugLED_Blink(3, 100); }

        if (!found) {
            I2C_MasterDeinit(I2C_INST);
            delay_ms(10);

            PORT_SetPinMux(I2C1_SCL_PORT, I2C1_SCL_PIN, kPORT_MuxAsGpio);
            gpio_pin_config_t scl_gpio = {kGPIO_DigitalOutput, 1};
            GPIO_PinInit(GPIOC, I2C1_SCL_PIN, &scl_gpio);

            for (int pulse = 0; pulse < 9; pulse++) {
                GPIO_PortClear(GPIOC, 1U << I2C1_SCL_PIN); delay_us(5);
                GPIO_PortSet(GPIOC,   1U << I2C1_SCL_PIN); delay_us(5);
            }

            PORT_SetPinConfig(I2C1_SCL_PORT, I2C1_SCL_PIN,
                &(port_pin_config_t){
                    kPORT_PullUp, kPORT_SlowSlewRate,
                    kPORT_PassiveFilterDisable, kPORT_OpenDrainEnable,
                    kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister
                });
            delay_ms(10);
        }
    }

    if (!found) { g_oled_ready = 0; DebugLED_Off(); return; }

    g_oled_ready = 1;
    delay_ms(WINDOWS_I2C_DELAY_MS);

    ssd1306_init();
    delay_ms(100);

    ssd1306_clear_page(0);
    ssd1306_clear_page(2);
    ssd1306_clear_page(4);
    ssd1306_clear_page(6);

    ssd1306_print(0, 0, "System Init OK");
    ssd1306_print(2, 0, "Ready");

    DebugLED_Off();
}

static void OLED_Show(uint8_t t, uint8_t h, bool dht_ok,
                       const char *s_txt, bool flame, int16_t ir_temp)
{
    if (!g_oled_ready) return;

    static uint32_t last_update = 0;
    uint32_t now = millis();
    if (now - last_update < 100) return;
    last_update = now;

    char line1[24], line2[24], line3[24], line4[24];

    ssd1306_clear_page(0);
    ssd1306_clear_page(2);
    ssd1306_clear_page(4);
    ssd1306_clear_page(6);

    if (dht_ok) {
        snprintf(line1, sizeof(line1), "T:%02uC H:%02u%%      ", t, h);
    } else {
        snprintf(line1, sizeof(line1), "T:--C H:--%%      ");
    }

    if (ir_temp != INT16_MIN) {
        int16_t ir_whole = ir_temp / 10;
        int16_t ir_frac  = ir_temp % 10;
        if (ir_frac < 0) ir_frac = -ir_frac;
        snprintf(line2, sizeof(line2), "IR:%d.%dC         ", ir_whole, ir_frac);
    } else {
        snprintf(line2, sizeof(line2), "IR:--C           ");
    }

    snprintf(line3, sizeof(line3), "S:%s             ", s_txt);
    snprintf(line4, sizeof(line4), "F:%s             ", flame ? "DETECT" : "NONE  ");

    ssd1306_print(0, 0, line1);
    ssd1306_print(2, 0, line2);
    ssd1306_print(4, 0, line3);
    ssd1306_print(6, 0, line4);

    DebugLED_Toggle();
}

/* ===================== Main ===================== */
int main(void)
{
    DWT_Init();
    delay_ms(WINDOWS_INIT_DELAY_MS);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    DWT_Init();

    DebugLED_Init();
    DebugLED_Blink(1, 500);

    InitGPIO();
    InitADC();

    delay_ms(WINDOWS_INIT_DELAY_MS);

    OLED_Init();

    delay_ms(WINDOWS_INIT_DELAY_MS);

    /* Init MLX90614 on same I2C bus as OLED */
    MLX90614_Init();

    delay_ms(WINDOWS_INIT_DELAY_MS);

    WIFI_UART_Init();

    delay_ms(2000);
    DebugLED_Blink(2, 200);

    uint16_t hum_f4  = 0;
    bool hum_init    = false;

    enum { S_NRM=0, S_WRN=1, S_DNG=2 } smoke_level = S_NRM;

    bool temp_warn40 = false;
    bool hum_warn80  = false;
    bool pump_on     = false;
    bool fan_on      = false;
    bool flame       = false;

    uint8_t last_temp   = 0;
    uint8_t last_hum    = 0;
    bool    last_dht_ok = false;
    int16_t ir_temp     = INT16_MIN;

    uint32_t last_sample_ms = millis();
    uint32_t last_heartbeat = millis();

    for (;;)
    {
        Buzzer_Tick();

        uint32_t now = millis();

        if (now - last_heartbeat >= 2000) {
            last_heartbeat = now;
            DebugLED_Blink(1, 50);
        }

        if ((now - last_sample_ms) >= 1000U)
        {
            last_sample_ms = now;

            flame   = Flame_Detected();
            ir_temp = MLX90614_ReadObjTemp();

            uint32_t smoke_mv = ReadSmoke_mV();

            if (smoke_level == S_NRM) {
                if (smoke_mv >= SMOKE_DANGER_MV)    smoke_level = S_DNG;
                else if (smoke_mv >= SMOKE_WARN_MV) smoke_level = S_WRN;
            } else if (smoke_level == S_WRN) {
                if (smoke_mv >= SMOKE_DANGER_MV)                      smoke_level = S_DNG;
                else if (smoke_mv < (SMOKE_WARN_MV - SMOKE_HYST_MV)) smoke_level = S_NRM;
            } else {
                if (smoke_mv < (SMOKE_DANGER_MV - SMOKE_HYST_MV)) {
                    if (smoke_mv >= SMOKE_WARN_MV) smoke_level = S_WRN;
                    else                           smoke_level = S_NRM;
                }
            }

            uint8_t temp1=0, hum1=0, temp2=0, hum2=0;
            bool ok = false;

            uint8_t e1 = dht11_read(&temp1, &hum1);
            delay_ms(60);
            uint8_t e2 = dht11_read(&temp2, &hum2);

            if (e2 == 0)      { last_temp = temp2; last_hum = hum2; ok = true; }
            else if (e1 == 0) { last_temp = temp1; last_hum = hum1; ok = true; }
            else ok = false;

            last_dht_ok = ok;

            uint8_t hum_show = last_hum;

            if (ok)
            {
                if (!hum_init) {
                    hum_f4   = (uint16_t)last_hum * 4U;
                    hum_init = true;
                } else {
                    hum_f4 = (uint16_t)((3U * hum_f4 + (uint16_t)last_hum * 4U) / 4U);
                }
                hum_show = (uint8_t)(hum_f4 / 4U);

                if (!temp_warn40) { if (last_temp >= TEMP_WARN1_C) temp_warn40 = true; }
                else              { if (last_temp <= (TEMP_WARN1_C - TEMP_HYST_C)) temp_warn40 = false; }

                if (!hum_warn80) { if (hum_show >= HUM_WARN1_PCT) hum_warn80 = true; }
                else             { if (hum_show <= (HUM_WARN1_PCT - HUM_HYST_PCT)) hum_warn80 = false; }

                if (!pump_on) {
                    if (last_temp >= PUMP_ON_TEMP_C || flame) { pump_on = true; Pump_Set(true); }
                } else {
                    if (last_temp <= PUMP_OFF_TEMP_C && !flame) { pump_on = false; Pump_Set(false); }
                }

                if (!fan_on) {
                    if (last_temp >= FAN_ON_TEMP_C) { fan_on = true;  Fan_Set(true);  }
                } else {
                    if (last_temp <= FAN_OFF_TEMP_C) { fan_on = false; Fan_Set(false); }
                }
            }
            else
            {
                temp_warn40 = false;
                hum_warn80  = false;
                pump_on     = false;
                fan_on      = false;
                Pump_Set(false);
                Fan_Set(false);
            }

            const char *s_txt = "NRM";
            if (smoke_level == S_DNG)      s_txt = "DNG";
            else if (smoke_level == S_WRN) s_txt = "WRN";

            OLED_Show(last_temp, hum_show, last_dht_ok, s_txt, flame, ir_temp);
            WIFI_UART_SendLine(last_temp, hum_show, s_txt, flame, ir_temp);

            bool danger = false;
            bool warn   = false;

            if (flame)                                      danger = true;
            if (smoke_level == S_DNG)                       danger = true;
            if (last_dht_ok && last_temp >= TEMP_DANGER_C)  danger = true;

            if (!danger) {
                if (smoke_level == S_WRN) warn = true;
                if (last_dht_ok && (temp_warn40 || hum_warn80)) warn = true;
            }

            if (danger)    Buzzer_SetMode(BUZ_DANGER_ON);
            else if (warn) Buzzer_SetMode(BUZ_WARN_BEEP);
            else           Buzzer_SetMode(BUZ_OFF);
        }

        delay_ms(10);
    }
}
