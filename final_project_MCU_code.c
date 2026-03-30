/*
 * =============================================================================
 * FILE:    main.c
 * PROJECT: SEP600 – Environmental Safety Monitor for Vulnerable Individuals
 * COURSE:  SEP600, Seneca Polytechnic, Winter 2026
 * TEAM:    Amirhossein Khonsari, Cody
 *
 * DESCRIPTION:
 *   Central firmware for the NXP Freedom K66F (180 MHz ARM Cortex-M4F).
 *   Reads seven sensors, drives three actuators, updates an OLED display,
 *   and streams a JSON telemetry packet to an ESP8266 co-processor via UART.
 *
 * SENSORS:
 *   DHT11       – Temperature & Humidity (GPIO bit-bang, PTC16)
 *   MQ-2        – Smoke / combustible gas (ADC1 CH10, DO on PTC0)
 *   MQ-135      – Air quality / CO2      (ADC1 CH11, DO on PTC1)
 *   KY-026      – Flame (IR photodiode)  (ADC1 CH12, DO on PTB19)
 *   MLX90614    – IR thermometer          (I2C0, PTB2/PTB3, addr 0x5A)
 *   FXOS8700CQ  – Accelerometer           (I2C0, PTD8/PTD9, addr 0x1D)
 *   RCWL-0516   – Microwave radar         (GPIO input, PTC6)
 *
 * ACTUATORS:
 *   Buzzer      – Graduated audio alert (GPIO, PTC8)
 *   Water Pump  – Cooling / suppression  (GPIO, PTD2, relay)
 *   Fan         – Ventilation            (GPIO, PTD3)
 *
 * DISPLAY:
 *   SSD1306 OLED 128x64 – Local status display (I2C1, PTC10/PTC11)
 *
 * COMMS:
 *   ESP8266 via UART1 (PTC4=TX, PTC3=RX, 9600 baud)
 *   JSON sent every 1 second:
 *   {"T":n,"H":n,"S":n,"F":n,"IR":n,"AQ":"str",
 *    "AX":n,"AY":n,"AZ":n,"RADAR":n,"ts":n}
 *
 * I2C0 PIN-MUX SHARING:
 *   The MLX90614 and FXOS8700CQ share I2C0 but use DIFFERENT physical pins.
 *   A software mutex (g_i2c0_owner) and pin tristate strategy switch between
 *   PTB2/PTB3 (MLX) and PTD8/PTD9 (FXOS) — the idle pair is tristated to
 *   prevent bus contention. This was the fix for the I2C conflict described
 *   in the milestone report.
 *
 * AI DECLARATION:
 *   All inline comments and the file header in this file were written with
 *   assistance from Claude (Anthropic) for documentation and grammar purposes.
 *   All hardware decisions, pin assignments, firmware logic, bug fixes
 *   (DWT init ordering, I2C pin-mux mutex, DHT11 dual-read, MQ warm-up),
 *   and system architecture were designed and implemented by the project team. AI
 *   was used in the process to help improve code and fix bugs.
 *   AI was also used to speed up the development process and help debug/bug fix
 *   [Claude – AI used for commenting and documentation only]
 * =============================================================================
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

/* =============================================================================
   Section 1: Pin & Peripheral Definitions
   All pin assignments are documented here for easy cross-reference with the
   hardware schematic. Changing a pin requires updating only this section.
   ============================================================================= */

/* Onboard debug LED – used for boot status blinks and heartbeat */
#define LED_PORT            PORTC
#define LED_GPIO            GPIOC
#define LED_PIN             17U

/* ADC1: shared by MQ-2, MQ-135, and Flame sensor analog outputs */
#define ADC_BASE            ADC1
#define ADC_GROUP           0U
#define MQ2_AO_CH           10U     /* MQ-2 analog smoke level          */
#define MQ135_AO_CH         11U     /* MQ-135 analog air quality level  */
#define FLAME_AO_CH         12U     /* KY-026 analog flame proximity    */
#define ADC_VREF_MV         3300U   /* 3.3V reference voltage in mV     */
#define ADC_MAX_COUNTS      4095U   /* 12-bit ADC full scale            */

/* Digital Output (DO) pins: pulled LOW when sensor threshold is exceeded.
   These are used to detect whether a sensor module is physically present. */
#define MQ2_DO_PORT         PORTC
#define MQ2_DO_GPIO         GPIOC
#define MQ2_DO_PIN          0U
#define MQ135_DO_PORT       PORTC
#define MQ135_DO_GPIO       GPIOC
#define MQ135_DO_PIN        1U
#define FLAME_DO_PORT       PORTB
#define FLAME_DO_GPIO       GPIOB
#define FLAME_DO_PIN        19U
#define FLAME_DETECT_MV     500U    /* ADC voltage below this = flame confirmed */

/*
 * RCWL-0516 Microwave Radar
 * Connected to PTC6 with pull-down. Output is 3.3V logic safe for direct
 * connection to K66F GPIO. HIGH = human presence detected, LOW = clear.
 * No protocol overhead — single GPIO read per sensor cycle.
 */
#define RADAR_PORT          PORTC
#define RADAR_GPIO          GPIOC
#define RADAR_PIN           6U

/* DHT11 single-wire bus pin – direction is toggled in software during reads */
#define DHT_PORT            PORTC
#define DHT_GPIO            GPIOC
#define DHT_PIN             16U

/* Actuator GPIO pins */
#define BUZ_PORT            PORTC
#define BUZ_GPIO            GPIOC
#define BUZ_PIN             8U      /* Buzzer – active HIGH                  */

#define PUMP_PORT           PORTD
#define PUMP_GPIO           GPIOD
#define PUMP_PIN            2U      /* Water pump relay – active HIGH        */

#define FAN_PORT            PORTD
#define FAN_GPIO            GPIOD
#define FAN_PIN             3U      /* Ventilation fan – active HIGH         */

/* UART1 – ESP8266 communication */
#define WIFI_UART           UART1
#define WIFI_UART_CLK       kCLOCK_Uart1
#define WIFI_UART_PORT      PORTC
#define WIFI_UART_TX_PIN    4U      /* K66F TX -> ESP8266 RX */
#define WIFI_UART_RX_PIN    3U      /* K66F RX <- ESP8266 TX */
#define WIFI_UART_BAUD      9600U

/*
 * I2C0 Shared Bus – Pin-Mux Switching
 *
 * Problem: MLX90614 (IR thermometer) and FXOS8700CQ (accelerometer) both
 * require I2C but have different physical pin connections. Placing both on
 * the same I2C lines caused bus contention and corrupted readings.
 *
 * Fix: I2C0 peripheral is kept active, but the pin routing is switched
 * dynamically. The idle pin pair is set to tristate (high-impedance) so it
 * cannot interfere with the active transaction. A mutex variable (g_i2c0_owner)
 * prevents redundant switching and serializes bus access.
 *
 * MLX90614:   SCL -> PTB2 (G12), SDA -> PTB3 (G11), I2C addr 0x5A
 * FXOS8700CQ: SCL -> PTD8 (C9),  SDA -> PTD9 (B9),  I2C addr 0x1D
 */
#define SHARED_I2C          I2C0
#define SHARED_I2C_CLK      kCLOCK_I2c0
#define MLX_SCL_PORT        PORTB
#define MLX_SDA_PORT        PORTB
#define MLX_SCL_PIN         2U
#define MLX_SDA_PIN         3U
#define FXOS_SCL_PORT       PORTD
#define FXOS_SDA_PORT       PORTD
#define FXOS_SCL_PIN        8U
#define FXOS_SDA_PIN        9U

/* MLX90614 register and retry config */
#define MLX_ADDR            0x5AU
#define MLX_REG_TOBJ1       0x07U   /* Object temperature register           */
#define MLX_RETRY_S         3U      /* Seconds between reconnect attempts     */

/* FXOS8700CQ register map (accelerometer-only mode) */
#define FXOS_ADDR           0x1DU
#define FXOS_OUT_X_MSB      0x01U   /* First XYZ output register             */
#define FXOS_WHO_AM_I       0x0DU   /* Identity register – must read 0xC7    */
#define FXOS_XYZ_DATA_CFG   0x0EU   /* Sensitivity config                    */
#define FXOS_CTRL_REG1      0x2AU   /* Mode control (standby / active)       */
#define FXOS_CTRL_REG2      0x2BU   /* Power mode                            */
#define FXOS_EXPECTED_WHO   0xC7U   /* Expected WHO_AM_I value               */

/* OLED SSD1306 on its own I2C1 bus – no mux switching needed */
#define OLED_I2C            I2C1
#define OLED_I2C_CLK        kCLOCK_I2c1
#define OLED_SCL_PORT       PORTC
#define OLED_SDA_PORT       PORTC
#define OLED_SCL_PIN        10U
#define OLED_SDA_PIN        11U
#define OLED_ADDR_3C        0x3CU   /* Common SSD1306 address (SA0=0)        */
#define OLED_ADDR_3D        0x3DU   /* Alternate address (SA0=1)             */
#define OLED_CTRL_CMD       0x00U   /* I2C control byte: command follows     */
#define OLED_CTRL_DATA      0x40U   /* I2C control byte: data follows        */
#define OLED_WATCHDOG_MS    500U    /* OLED health-check interval            */

/* =============================================================================
   Section 2: Safety Thresholds -- Section generated mostly by AI
   All thresholds use hysteresis to prevent rapid on/off toggling at the
   boundary condition (e.g., smoke bouncing between WARN and NRM).
   ============================================================================= */

#define SMOKE_WARN_MV       1600U   /* MQ-2 mV: enter WARNING state          */
#define SMOKE_DANGER_MV     2000U   /* MQ-2 mV: enter DANGER state           */
#define SMOKE_HYST_MV       50U     /* Hysteresis band for smoke transitions  */
#define AQ_GOOD_MV          800U    /* MQ-135 mV: GOOD air quality           */
#define AQ_FAIR_MV          1500U   /* MQ-135 mV: FAIR air quality           */
#define AQ_POOR_MV          2500U   /* MQ-135 mV: POOR air quality           */
#define TEMP_WARN_C         40U     /* DHT11 °C: temperature warning level   */
#define TEMP_DANGER_C       60U     /* DHT11 °C: temperature danger level    */
#define TEMP_HYST_C         1U      /* Hysteresis for temperature warnings    */
#define HUM_WARN_PCT        80U     /* Humidity % warning threshold          */
#define HUM_HYST_PCT        2U      /* Hysteresis for humidity warnings       */
#define PUMP_ON_TEMP_C      26U     /* Pump activates above this temp        */
#define PUMP_OFF_TEMP_C     25U     /* Pump deactivates below this temp      */
#define FAN_ON_TEMP_C       27U     /* Fan activates above this temp         */
#define FAN_OFF_TEMP_C      26U     /* Fan deactivates below this temp       */
#define BEEP_ON_MS          80U     /* Buzzer ON duration in BEEP mode       */
#define BEEP_OFF_MS         420U    /* Buzzer OFF duration in BEEP mode      */
#define INIT_DELAY_MS       500U    /* Post-reset stabilization delay        */
#define I2C_SETTLE_MS       200U    /* I2C bus settle time after pin config  */

/* =============================================================================
   Section 3: Global State Variables
   ============================================================================= */

/* MLX90614 connection tracking */
static bool     g_mlx_ok      = false; /* true when sensor is responding     */
static uint32_t g_mlx_retry_s = 0U;    /* uptime timestamp for next reconnect*/

/* FXOS8700CQ accelerometer state */
static bool    g_accel_ok  = false;    /* true when sensor passed WHO_AM_I   */
static int16_t g_ax_raw    = 0;        /* Raw 14-bit X axis reading          */
static int16_t g_ay_raw    = 0;        /* Raw 14-bit Y axis reading          */
static int16_t g_az_raw    = 0;        /* Raw 14-bit Z axis reading          */
static int16_t g_az_offset = 0;        /* Calibration offset for Z axis      */

/* OLED display state */
static bool    g_oled_ok   = false;    /* false triggers bus recovery        */
static uint8_t g_oled_addr = OLED_ADDR_3C; /* Detected at runtime            */

/* RCWL-0516 radar: updated each 1-second sample cycle */
static bool g_radar = false;

/*
 * I2C0 mutex: tracks which device pair currently owns the I2C0 peripheral.
 * I2C0_SelectMLX() and I2C0_SelectFXOS() check this before switching pins,
 * avoiding unnecessary tristate/activate cycles on every call.
 */
typedef enum { I2C0_OWNER_NONE=0, I2C0_OWNER_MLX, I2C0_OWNER_FXOS } i2c0_owner_t;
static volatile i2c0_owner_t g_i2c0_owner = I2C0_OWNER_NONE;

/* =============================================================================
   Section 4: DWT Cycle-Counter Timing -- AI used to help debug
   The ARM DWT (Data Watchpoint and Trace) cycle counter provides microsecond-
   accurate timing without consuming a hardware timer peripheral. It must be
   initialized as the very first operation in main() — placing it later caused
   the I2C OLED initialization failure on Windows (all early delays resolved
   to zero cycles before the counter was enabled).
   ============================================================================= */

static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* Enable trace unit     */
    DWT->CYCCNT = 0;                                  /* Reset counter         */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;            /* Start counting        */
}
static inline uint32_t DWT_Cycles(void) { return DWT->CYCCNT; }

/* Busy-wait delay using DWT cycle count – accurate to ±1 µs */
static void delay_us(uint32_t us) {
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    uint32_t start = DWT_Cycles();
    while ((DWT_Cycles() - start) < ticks) {}
}
static void delay_ms(uint32_t ms) { while (ms--) delay_us(1000U); }

/* Returns elapsed time in milliseconds since last DWT reset (wraps ~49 days) */
static uint32_t millis(void) { return DWT_Cycles() / (SystemCoreClock / 1000U); }

/* =============================================================================
   Section 5: Debug LED
   Used for boot sequence indication and runtime heartbeat blinks.
   ============================================================================= */

static void LED_Init(void) {
    CLOCK_EnableClock(kCLOCK_PortC);
    gpio_pin_config_t cfg = {kGPIO_DigitalOutput, 0};
    PORT_SetPinMux(LED_PORT, LED_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(LED_GPIO, LED_PIN, &cfg);
    GPIO_PortClear(LED_GPIO, 1U << LED_PIN);
}
static void LED_On(void)     { GPIO_PortSet(LED_GPIO,    1U << LED_PIN); }
static void LED_Off(void)    { GPIO_PortClear(LED_GPIO,  1U << LED_PIN); }
static void LED_Toggle(void) { GPIO_PortToggle(LED_GPIO, 1U << LED_PIN); }

/* Blink n times with ms ON/OFF period – used for init status feedback */
static void LED_Blink(uint32_t n, uint32_t ms) {
    for (uint32_t i = 0; i < n; i++) {
        LED_On(); delay_ms(ms); LED_Off(); delay_ms(ms);
    }
}

/* =============================================================================
   Section 6: Shared Pin Configurations -- partially generate by AI
   Reusable port_pin_config_t structs for common pin modes.
   ============================================================================= */

/* Pull-down input: safe default for sensor DO lines when module absent */
static const port_pin_config_t k_pulldown_pin = {
    kPORT_PullDown, kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
    kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister
};

/* I2C pin: open-drain with pull-up as required by I2C specification */
static const port_pin_config_t k_i2c_pin = {
    kPORT_PullUp, kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable, kPORT_OpenDrainEnable,
    kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister
};

/*
 * Tristate pin: completely disconnects an idle I2C0 pin pair from the bus.
 * Without this, the inactive pair's pull-up resistors would fight the active
 * pair during I2C transactions, causing data corruption.
 */
static const port_pin_config_t k_tristate_pin = {
    kPORT_PullDisable, kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
    kPORT_LowDriveStrength, kPORT_PinDisabledOrAnalog, kPORT_UnlockRegister
};

/* =============================================================================
   Section 7: I2C0 Pin-Mux Switching (Mutex-Protected) -- Generated by AI because this was giving us trouble
   Implements the hardware fix for the accelerometer / IR thermometer I2C
   conflict. Each function tristates the idle pair and activates the target pair.
   The g_i2c0_owner guard prevents redundant switching on consecutive calls
   to the same device.
   ============================================================================= */

static void I2C0_SelectMLX(void) {
    if (g_i2c0_owner == I2C0_OWNER_MLX) return; /* Already selected — no-op  */
    /* Tristate FXOS pins first to avoid bus fight during transition */
    PORT_SetPinConfig(FXOS_SCL_PORT, FXOS_SCL_PIN, &k_tristate_pin);
    PORT_SetPinConfig(FXOS_SDA_PORT, FXOS_SDA_PIN, &k_tristate_pin);
    /* Activate MLX pins */
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinConfig(MLX_SCL_PORT, MLX_SCL_PIN, &k_i2c_pin);
    PORT_SetPinConfig(MLX_SDA_PORT, MLX_SDA_PIN, &k_i2c_pin);
    delay_us(10); /* Allow pull-ups to settle before transaction */
    g_i2c0_owner = I2C0_OWNER_MLX;
}

static void I2C0_SelectFXOS(void) {
    if (g_i2c0_owner == I2C0_OWNER_FXOS) return; /* Already selected — no-op */
    /* Tristate MLX pins first */
    PORT_SetPinConfig(MLX_SCL_PORT, MLX_SCL_PIN, &k_tristate_pin);
    PORT_SetPinConfig(MLX_SDA_PORT, MLX_SDA_PIN, &k_tristate_pin);
    /* Activate FXOS pins */
    CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinConfig(FXOS_SCL_PORT, FXOS_SCL_PIN, &k_i2c_pin);
    PORT_SetPinConfig(FXOS_SDA_PORT, FXOS_SDA_PIN, &k_i2c_pin);
    delay_us(10);
    g_i2c0_owner = I2C0_OWNER_FXOS;
}

/* =============================================================================
   Section 8: Actuator & Sensor GPIO Control
   ============================================================================= */

static void Buzzer_Set(bool on) {
    if (on) GPIO_PortSet(BUZ_GPIO, 1U << BUZ_PIN);
    else    GPIO_PortClear(BUZ_GPIO, 1U << BUZ_PIN);
}
static void Pump_Set(bool on) {
    if (on) GPIO_PortSet(PUMP_GPIO, 1U << PUMP_PIN);
    else    GPIO_PortClear(PUMP_GPIO, 1U << PUMP_PIN);
}
static void Fan_Set(bool on) {
    if (on) GPIO_PortSet(FAN_GPIO, 1U << FAN_PIN);
    else    GPIO_PortClear(FAN_GPIO, 1U << FAN_PIN);
}

/* Sensor presence detection via DO pin:
   HIGH = module present and threshold exceeded (active-high open-collector) */
static bool MQ2_Present(void)    { return ((MQ2_DO_GPIO->PDIR   >> MQ2_DO_PIN)   & 1U) == 1U; }
static bool MQ135_Present(void)  { return ((MQ135_DO_GPIO->PDIR >> MQ135_DO_PIN) & 1U) == 1U; }
static bool Flame_Present(void)  { return ((FLAME_DO_GPIO->PDIR >> FLAME_DO_PIN) & 1U) == 1U; }

/* RCWL-0516: returns true when microwave Doppler detection is active */
static bool Radar_Detected(void) { return ((RADAR_GPIO->PDIR >> RADAR_PIN) & 1U) == 1U; }

static void InitGPIO(void) {
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);

    gpio_pin_config_t out = {kGPIO_DigitalOutput, 0};
    gpio_pin_config_t in  = {kGPIO_DigitalInput,  0};

    /* Buzzer: output, initially OFF */
    PORT_SetPinMux(BUZ_PORT,  BUZ_PIN,  kPORT_MuxAsGpio);
    GPIO_PinInit(BUZ_GPIO,    BUZ_PIN,  &out); Buzzer_Set(false);

    /* DHT11: starts as input; driver toggles direction during bit-bang reads */
    PORT_SetPinMux(DHT_PORT, DHT_PIN, kPORT_MuxAsGpio);
    DHT_GPIO->PDDR &= ~(1U << DHT_PIN);

    /* Water pump: output, initially OFF */
    PORT_SetPinMux(PUMP_PORT, PUMP_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(PUMP_GPIO,   PUMP_PIN, &out); Pump_Set(false);

    /* Fan: output, initially OFF */
    PORT_SetPinMux(FAN_PORT,  FAN_PIN,  kPORT_MuxAsGpio);
    GPIO_PinInit(FAN_GPIO,    FAN_PIN,  &out); Fan_Set(false);

    /* MQ-2, MQ-135, Flame DO: pull-down inputs */
    PORT_SetPinConfig(MQ2_DO_PORT,   MQ2_DO_PIN,   &k_pulldown_pin);
    GPIO_PinInit(MQ2_DO_GPIO,        MQ2_DO_PIN,   &in);
    PORT_SetPinConfig(MQ135_DO_PORT, MQ135_DO_PIN, &k_pulldown_pin);
    GPIO_PinInit(MQ135_DO_GPIO,      MQ135_DO_PIN, &in);
    PORT_SetPinConfig(FLAME_DO_PORT, FLAME_DO_PIN, &k_pulldown_pin);
    GPIO_PinInit(FLAME_DO_GPIO,      FLAME_DO_PIN, &in);

    /* RCWL-0516: pull-down ensures LOW when module is absent or unpowered */
    PORT_SetPinConfig(RADAR_PORT, RADAR_PIN, &k_pulldown_pin);
    GPIO_PinInit(RADAR_GPIO, RADAR_PIN, &in);

    delay_ms(50); /* Allow input pins to settle after pull configuration */
}

/* =============================================================================
   Section 9: Buzzer State Machine
   Three modes: OFF, BEEP (periodic), SOLID (continuous).
   Buzzer_Tick() must be called in the main loop to drive the BEEP timing.
   Priority (set in main loop): SOLID > BEEP > OFF.
   ============================================================================= */

typedef enum { BUZ_OFF=0, BUZ_BEEP=1, BUZ_SOLID=2 } buz_mode_t;
static struct { buz_mode_t mode; uint32_t next_ms; bool on; }
    g_buz = {BUZ_OFF, 0, false};

static void Buzzer_SetMode(buz_mode_t m) {
    if (g_buz.mode == m) return; /* Avoid resetting timing on same-mode calls */
    g_buz.mode = m; g_buz.on = false;
    g_buz.next_ms = millis(); Buzzer_Set(false);
}
static void Buzzer_Tick(void) {
    uint32_t now = millis();
    switch (g_buz.mode) {
        case BUZ_OFF:   Buzzer_Set(false); break;
        case BUZ_SOLID: Buzzer_Set(true);  break;
        case BUZ_BEEP:
            /* Toggle buzzer at BEEP_ON_MS / BEEP_OFF_MS intervals */
            if (now >= g_buz.next_ms) {
                g_buz.on = !g_buz.on; Buzzer_Set(g_buz.on);
                g_buz.next_ms = now + (g_buz.on ? BEEP_ON_MS : BEEP_OFF_MS);
            }
            break;
    }
}

/* =============================================================================
   Section 10: ADC1 Initialization and Reading
   12-bit single-ended, hardware averaging over 32 samples for noise rejection.
   Calibration is performed at startup for maximum accuracy.
   ============================================================================= */

static void ADC_Init_Local(void) {
    adc16_config_t cfg;
    CLOCK_EnableClock(kCLOCK_Adc1);
    ADC16_GetDefaultConfig(&cfg);
    cfg.resolution = kADC16_ResolutionSE12Bit;         /* 12-bit resolution   */
    ADC16_Init(ADC_BASE, &cfg);
    ADC16_EnableHardwareTrigger(ADC_BASE, false);       /* Software trigger    */
    ADC16_SetHardwareAverage(ADC_BASE, kADC16_HardwareAverageCount32); /* 32x avg */
    (void)ADC16_DoAutoCalibration(ADC_BASE);            /* Factory calibration */
}

/* Read ADC channel and convert raw count to millivolts */
static uint32_t ADC_ReadMV(uint32_t ch) {
    adc16_channel_config_t c = {0};
    c.channelNumber = ch;
    c.enableInterruptOnConversionCompleted = false;
    ADC16_SetChannelConfig(ADC_BASE, ADC_GROUP, &c);
    while (!(ADC16_GetChannelStatusFlags(ADC_BASE, ADC_GROUP)
             & kADC16_ChannelConversionDoneFlag)) {}
    return (ADC16_GetChannelConversionValue(ADC_BASE, ADC_GROUP)
            * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

/* Map MQ-135 ADC reading to human-readable air quality label */
static const char* AQ_Label(uint32_t mv) {
    if (mv < AQ_GOOD_MV) return "GOOD";
    if (mv < AQ_FAIR_MV) return "FAIR";
    if (mv < AQ_POOR_MV) return "POOR";
    return "BAD";
}

/* =============================================================================
   Section 11: DHT11 Bit-Bang Driver -- Generated partially by AI
   The DHT11 uses a proprietary single-wire protocol:
     1. Host pulls LOW for 18ms  (start signal)
     2. Host releases; sensor responds with 80µs LOW + 80µs HIGH
     3. 40 bits follow: 50µs LOW + 26-28µs HIGH = 0, 50µs LOW + 70µs HIGH = 1
     4. Byte 5 is a checksum = sum of bytes 1-4

   Dual-read strategy: Two reads are taken 60ms apart. The second result is
   preferred because the DHT11 has had time to fully reset and re-drive the
   line. This eliminated intermittent checksum failures caused by bus
   capacitance on longer wire runs slowing down pulse edges.
   ============================================================================= */

static inline void dht_dir_out(void) { DHT_GPIO->PDDR |=  (1U << DHT_PIN); }
static inline void dht_dir_in(void)  { DHT_GPIO->PDDR &= ~(1U << DHT_PIN); }
static inline void dht_low(void)     { DHT_GPIO->PCOR  =  (1U << DHT_PIN); }
static inline void dht_high(void)    { DHT_GPIO->PSOR  =  (1U << DHT_PIN); }
static inline uint8_t dht_pin(void)  { return (uint8_t)((DHT_GPIO->PDIR >> DHT_PIN) & 1U); }

/* Wait for pin to reach 'level', timeout after timeout_us microseconds */
static bool dht_wait(uint8_t level, uint32_t timeout_us) {
    uint32_t ticks = (SystemCoreClock / 1000000U) * timeout_us;
    uint32_t start = DWT_Cycles();
    while (dht_pin() != level)
        if ((DWT_Cycles() - start) > ticks) return false;
    return true;
}

/* Single DHT11 read attempt. Returns false on timeout or checksum failure. */
static bool DHT11_Read(uint8_t *tempC, uint8_t *hum) {
    uint8_t data[5] = {0};
    /* Start signal: pull LOW for 18ms, then release and wait for response */
    dht_dir_out(); dht_low(); delay_ms(18);
    dht_high(); delay_us(30); dht_dir_in();
    if (!dht_wait(0, 120)) return false; /* Wait for sensor LOW response      */
    if (!dht_wait(1, 120)) return false; /* Wait for sensor HIGH response     */
    if (!dht_wait(0, 120)) return false; /* Wait for data start               */
    /* Read 40 bits: sample at 35µs after rising edge to distinguish 0 vs 1  */
    for (int i = 0; i < 40; i++) {
        if (!dht_wait(1, 120)) return false;
        delay_us(35); /* Sample point: 0-bit is ~26µs, 1-bit is ~70µs       */
        if (dht_pin()) data[i / 8] |= (1U << (7 - (i % 8)));
        if (!dht_wait(0, 150)) return false;
    }
    /* Verify checksum (byte 5 = sum of bytes 1-4, lower 8 bits) */
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) return false;
    *hum = data[0]; *tempC = data[2];
    return true;
}

/* =============================================================================
   Section 12: I2C Helper Functions - Generated by AI
   Wrappers around the FSL I2C driver with timeout protection.
   A stuck bus (SCL held LOW by a device) is detected and cleared.
   ============================================================================= */

/* Blocking I2C transfer with timeout. Resets peripheral if bus is stuck. */
static status_t I2C_XferTimeout(I2C_Type *base, i2c_master_transfer_t *xfer,
                                 uint32_t timeout_ms) {
    uint32_t start = millis();
    /* Wait for bus to become free before initiating transfer */
    while (base->S & I2C_S_BUSY_MASK) {
        if ((millis() - start) > timeout_ms) {
            /* Bus stuck: reset I2C peripheral to clear the condition */
            base->C1 &= ~I2C_C1_IICEN_MASK; delay_us(10);
            base->C1 |=  I2C_C1_IICEN_MASK;
            if (base == OLED_I2C) g_oled_ok = false;
            return kStatus_Fail;
        }
    }
    status_t s = I2C_MasterTransferBlocking(base, xfer);
    if (base == OLED_I2C && s != kStatus_Success) g_oled_ok = false;
    return s;
}

/* Write len bytes to device addr on the given I2C bus */
static status_t I2C_Write(I2C_Type *base, uint8_t addr,
                           const uint8_t *data, size_t len) {
    i2c_master_transfer_t x; memset(&x, 0, sizeof(x));
    x.slaveAddress = addr; x.direction = kI2C_Write;
    x.data = (uint8_t *)data; x.dataSize = len;
    x.flags = kI2C_TransferDefaultFlag;
    return I2C_XferTimeout(base, &x, 10);
}

/* Write register address then read rxlen bytes from device */
static status_t I2C_ReadReg(I2C_Type *base, uint8_t addr,
                             uint8_t reg, uint8_t *rxbuf, size_t rxlen) {
    i2c_master_transfer_t x; memset(&x, 0, sizeof(x));
    x.slaveAddress = addr; x.direction = kI2C_Read;
    x.subaddress = reg; x.subaddressSize = 1;
    x.data = rxbuf; x.dataSize = rxlen;
    x.flags = kI2C_TransferDefaultFlag;
    return I2C_XferTimeout(base, &x, 10);
}

/* Attempt a zero-length write to check if device ACKs its address */
static bool I2C_Ping(I2C_Type *base, uint8_t addr) {
    i2c_master_transfer_t x; memset(&x, 0, sizeof(x));
    x.slaveAddress = addr; x.direction = kI2C_Write;
    x.data = NULL; x.dataSize = 0;
    x.flags = kI2C_TransferDefaultFlag;
    return I2C_XferTimeout(base, &x, 10) == kStatus_Success;
}

/* FXOS-specific two-byte write (register + value) */
static bool FXOS_Write8(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg, val};
    i2c_master_transfer_t x; memset(&x, 0, sizeof(x));
    x.slaveAddress = FXOS_ADDR; x.direction = kI2C_Write;
    x.data = tx; x.dataSize = 2;
    x.flags = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(SHARED_I2C, &x) == kStatus_Success;
}

/* Initialize the I2C0 peripheral (shared between MLX and FXOS) */
static void SharedI2C_PeripheralInit(void) {
    CLOCK_EnableClock(SHARED_I2C_CLK);
    i2c_master_config_t cfg; I2C_MasterGetDefaultConfig(&cfg);
    cfg.baudRate_Bps = 100000U; /* Standard mode 100 kHz */
    I2C_MasterInit(SHARED_I2C, &cfg, CLOCK_GetFreq(kCLOCK_BusClk));
    delay_ms(50);
}

/* =============================================================================
   Section 13: MLX90614 IR Thermometer Driver -- AI used for some logical errors
   Uses I2C0 via PTB2/PTB3 (selected via I2C0_SelectMLX).
   Raw reading is in units of 0.02°C; conversion: (raw * 0.02) - 273.15
   Result is returned as integer tenths of degrees Celsius (e.g. 236 = 23.6°C).
   ============================================================================= */

static void MLX90614_Init(void) {
    I2C0_SelectMLX(); /* Switch I2C0 pins to PTB2/PTB3 */
    g_mlx_ok = I2C_Ping(SHARED_I2C, MLX_ADDR);
    if (g_mlx_ok) LED_Blink(4, 50); /* 4 quick blinks = MLX found */
}

/* Periodically check MLX connection and attempt reconnect after MLX_RETRY_S */
static void MLX90614_CheckConnection(uint32_t uptime_s) {
    if (g_mlx_ok) {
        I2C0_SelectMLX();
        if (!I2C_Ping(SHARED_I2C, MLX_ADDR)) {
            g_mlx_ok = false; g_mlx_retry_s = uptime_s + MLX_RETRY_S;
        }
    } else if (uptime_s >= g_mlx_retry_s) {
        g_mlx_retry_s = uptime_s + MLX_RETRY_S;
        I2C0_SelectMLX();
        if (I2C_Ping(SHARED_I2C, MLX_ADDR)) { g_mlx_ok = true; LED_Blink(4, 50); }
    }
}

/* Read object temperature register. Returns INT16_MIN on failure. */
static int16_t MLX90614_ReadTemp(void) {
    if (!g_mlx_ok) return INT16_MIN;
    I2C0_SelectMLX();
    uint8_t buf[3] = {0};
    if (I2C_ReadReg(SHARED_I2C, MLX_ADDR, MLX_REG_TOBJ1, buf, 3) != kStatus_Success)
        return INT16_MIN;
    /* Raw format: 15-bit value in units of 0.02K. Convert to tenths of °C. */
    uint16_t raw = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7FU) << 8);
    return (int16_t)(((int32_t)raw * 2 - 27315) / 10);
}

/* =============================================================================
   Section 14: FXOS8700CQ Accelerometer Driver -- written with help from lab code
   Uses I2C0 via PTD8/PTD9 (selected via I2C0_SelectFXOS).
   Configured in accelerometer-only mode, ±2g range (14-bit, 4096 LSB/g).
   Offset calibration subtracts steady-state Z noise for accurate delta readings.
   ============================================================================= */

/* Read raw 14-bit XYZ registers (6 bytes from FXOS_OUT_X_MSB) */
static void FXOS_ReadXYZ(void) {
    if (!g_accel_ok) return;
    I2C0_SelectFXOS(); /* Switch I2C0 pins to PTD8/PTD9 */
    uint8_t buf[6] = {0};
    if (I2C_ReadReg(SHARED_I2C, FXOS_ADDR, FXOS_OUT_X_MSB, buf, 6) != kStatus_Success) return;
    /* Right-shift 2 because data is left-aligned 14-bit in 16-bit registers */
    g_ax_raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]) >> 2;
    g_ay_raw = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]) >> 2;
    g_az_raw = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]) >> 2;
}

/* Average 4 readings to reduce quantization noise (called at 20 Hz) */
static void FXOS_ReadXYZ_Averaged(void) {
    if (!g_accel_ok) return;
    int32_t sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < 4; i++) {
        FXOS_ReadXYZ(); sx += g_ax_raw; sy += g_ay_raw; sz += g_az_raw; delay_ms(5);
    }
    g_ax_raw = (int16_t)(sx / 4); g_ay_raw = (int16_t)(sy / 4); g_az_raw = (int16_t)(sz / 4);
}

/* Measure static Z offset at startup (device flat = 1g = 4096 LSB expected) */
static void FXOS_CalibrateOffsets(void) {
    if (!g_accel_ok) return;
    int32_t sx = 0, sy = 0, sz = 0;
    delay_ms(300); /* Allow sensor to reach steady state */
    for (int i = 0; i < 16; i++) { FXOS_ReadXYZ(); delay_ms(10); } /* Discard warmup */
    for (int i = 0; i < 64; i++) {
        FXOS_ReadXYZ(); sx += g_ax_raw; sy += g_ay_raw; sz += g_az_raw; delay_ms(10);
    }
    /* Z offset = measured Z mean minus expected 1g = 4096 LSB */
    g_az_offset = (int16_t)((sz / 64) - 4096); (void)sx; (void)sy;
}

/* Initialize FXOS8700CQ: verify WHO_AM_I, configure accelerometer-only mode */
static void FXOS_Init(void) {
    g_accel_ok = false;
    I2C0_SelectFXOS();
    uint8_t who = 0;
    /* Verify device identity before writing configuration registers */
    if (I2C_ReadReg(SHARED_I2C, FXOS_ADDR, FXOS_WHO_AM_I, &who, 1) != kStatus_Success
        || who != FXOS_EXPECTED_WHO) return;
    /* Place in standby, reset, configure ±2g range, then activate */
    (void)FXOS_Write8(FXOS_CTRL_REG1,    0x00U); delay_ms(10); /* Standby        */
    (void)FXOS_Write8(FXOS_CTRL_REG2,    0x40U); delay_ms(50); /* Auto-sleep off */
    (void)FXOS_Write8(FXOS_CTRL_REG1,    0x00U); delay_ms(10); /* Standby again  */
    (void)FXOS_Write8(FXOS_XYZ_DATA_CFG, 0x00U); delay_ms(5);  /* ±2g range      */
    (void)FXOS_Write8(FXOS_CTRL_REG1,    0x01U); delay_ms(20); /* Active mode    */
    g_accel_ok = true; LED_Blink(4, 50); FXOS_CalibrateOffsets();
}

/* Convert raw 14-bit reading to milli-g (1000 = 1g) */
static int16_t raw_to_mg(int16_t raw) { return (int16_t)(((int32_t)raw * 1000) / 4096); }

/* =============================================================================
   Section 15: OLED SSD1306 Driver
   Uses I2C1 (PTC10/PTC11) — independent bus, no mux switching needed.
   Custom 5x7 pixel font baked in as a lookup table (k_font[]).
   OLED pages used:
     Page 0: T:xxC  H:xx%
     Page 2: IR:xx.xC  AQ:GOOD
     Page 4: S:NRM  F:NO
     Page 6: RADAR:YES / RADAR:NO
   ============================================================================= */

static void oled_cmd(uint8_t c) {
    uint8_t b[2] = {OLED_CTRL_CMD, c}; I2C_Write(OLED_I2C, g_oled_addr, b, 2);
}
static void oled_cmd_list(const uint8_t *cmds, size_t n) {
    uint8_t b[64]; if (n + 1 > sizeof(b)) return;
    b[0] = OLED_CTRL_CMD; memcpy(&b[1], cmds, n);
    I2C_Write(OLED_I2C, g_oled_addr, b, n + 1);
}
static void oled_data(const uint8_t *d, size_t n) {
    uint8_t b[17];
    while (n) {
        size_t chunk = (n > 16) ? 16 : n;
        b[0] = OLED_CTRL_DATA; memcpy(&b[1], d, chunk);
        I2C_Write(OLED_I2C, g_oled_addr, b, chunk + 1);
        d += chunk; n -= chunk;
    }
}
/* Set OLED cursor to page (row of 8 pixels) and column */
static void oled_set_cursor(uint8_t page, uint8_t col) {
    oled_cmd(0xB0U | (page & 0x07U));          /* Page address              */
    oled_cmd(0x00U | (col  & 0x0FU));          /* Column lower nibble       */
    oled_cmd(0x10U | ((col >> 4) & 0x0FU));    /* Column upper nibble       */
}
static void oled_clear_page(uint8_t page) {
    uint8_t z[128]; memset(z, 0, 128); oled_set_cursor(page, 0); oled_data(z, 128);
}

/* SSD1306 initialization command sequence */
static void oled_hw_init(void) {
    const uint8_t seq[] = {
        0xAE,       /* Display OFF                */
        0xD5, 0x80, /* Clock divide ratio         */
        0xA8, 0x3F, /* Multiplex ratio 64         */
        0xD3, 0x00, /* Display offset 0           */
        0x40,       /* Start line 0               */
        0x8D, 0x14, /* Charge pump ON             */
        0x20, 0x02, /* Page addressing mode       */
        0xA0,       /* Segment remap normal       */
        0xC0,       /* COM scan direction normal  */
        0xDA, 0x12, /* COM pins config            */
        0x81, 0xCF, /* Contrast                   */
        0xD9, 0xF1, /* Pre-charge period          */
        0xDB, 0x40, /* VCOMH deselect level       */
        0xA4,       /* Entire display RAM         */
        0xA6,       /* Normal (non-inverted)      */
        0xAF        /* Display ON                 */
    };
    oled_cmd_list(seq, sizeof(seq));
}

/* 5-column bitmaps for each printable character (5x8 pixel font) */
typedef struct { char ch; uint8_t col[5]; } glyph_t;
static const glyph_t k_font[] = {
    {' ',{0x00,0x00,0x00,0x00,0x00}},{':',{0x00,0x36,0x36,0x00,0x00}},
    {'.',{0x00,0x60,0x60,0x00,0x00}},{'%',{0x62,0x64,0x08,0x13,0x23}},
    {'-',{0x08,0x08,0x08,0x08,0x08}},{'+',{0x08,0x08,0x3E,0x08,0x08}},
    {'/',{0x60,0x10,0x08,0x04,0x03}},
    {'0',{0x3E,0x41,0x41,0x41,0x3E}},{'1',{0x00,0x42,0x7F,0x40,0x00}},
    {'2',{0x62,0x51,0x49,0x49,0x46}},{'3',{0x22,0x41,0x49,0x49,0x36}},
    {'4',{0x18,0x14,0x12,0x7F,0x10}},{'5',{0x2F,0x49,0x49,0x49,0x31}},
    {'6',{0x3E,0x49,0x49,0x49,0x32}},{'7',{0x01,0x71,0x09,0x05,0x03}},
    {'8',{0x36,0x49,0x49,0x49,0x36}},{'9',{0x26,0x49,0x49,0x49,0x3E}},
    {'A',{0x7E,0x11,0x11,0x11,0x7E}},{'B',{0x7F,0x49,0x49,0x49,0x36}},
    {'C',{0x3E,0x41,0x41,0x41,0x22}},{'D',{0x7F,0x41,0x41,0x22,0x1C}},
    {'E',{0x7F,0x49,0x49,0x49,0x41}},{'F',{0x7F,0x09,0x09,0x09,0x01}},
    {'G',{0x3E,0x41,0x49,0x29,0x1E}},{'H',{0x7F,0x08,0x08,0x08,0x7F}},
    {'I',{0x00,0x41,0x7F,0x41,0x00}},{'K',{0x7F,0x08,0x14,0x22,0x41}},
    {'L',{0x7F,0x40,0x40,0x40,0x40}},{'M',{0x7F,0x02,0x0C,0x02,0x7F}},
    {'N',{0x7F,0x06,0x18,0x60,0x7F}},{'O',{0x3E,0x41,0x41,0x41,0x3E}},
    {'P',{0x7F,0x09,0x09,0x09,0x06}},{'Q',{0x1C,0x22,0x26,0x22,0x1D}},
    {'R',{0x7F,0x09,0x19,0x29,0x46}},{'S',{0x46,0x49,0x49,0x49,0x31}},
    {'T',{0x01,0x01,0x7F,0x01,0x01}},{'U',{0x3F,0x40,0x40,0x40,0x3F}},
    {'W',{0x7F,0x20,0x18,0x20,0x7F}},{'X',{0x63,0x14,0x08,0x14,0x63}},
    {'Y',{0x03,0x04,0x78,0x04,0x03}},{'Z',{0x61,0x51,0x49,0x45,0x43}},
};

/* Find glyph bitmap by character; returns space bitmap if not found */
static const uint8_t* glyph_lookup(char c) {
    for (unsigned i = 0; i < sizeof(k_font)/sizeof(k_font[0]); i++)
        if (k_font[i].ch == c) return k_font[i].col;
    return k_font[0].col; /* Default to space */
}

/* Print a string at given OLED page and column (6 pixels per character) */
static void oled_print(uint8_t page, uint8_t col, const char *s) {
    oled_set_cursor(page, col);
    while (*s) { oled_data(glyph_lookup(*s++), 5); uint8_t gap = 0; oled_data(&gap, 1); }
}

/*
 * OLED Bus Recovery
 * If the SDA line gets stuck LOW (e.g. after a power glitch mid-transaction),
 * the I2C peripheral cannot recover itself. This routine:
 *  1. De-initializes the I2C peripheral
 *  2. Clocks SCL 9 times manually (GPIO mode) to force any stuck device to
 *     release SDA (per I2C specification recovery procedure)
 *  3. Re-initializes the I2C peripheral and re-draws the OLED if successful
 */
static void OLED_BusRecover(void) {
    I2C_MasterDeinit(OLED_I2C); delay_ms(10);
    PORT_SetPinMux(OLED_SCL_PORT, OLED_SCL_PIN, kPORT_MuxAsGpio);
    gpio_pin_config_t scl_out = {kGPIO_DigitalOutput, 1};
    GPIO_PinInit(GPIOC, OLED_SCL_PIN, &scl_out);
    /* 9 manual SCL pulses: enough to clock out any stuck device state */
    for (int p = 0; p < 9; p++) {
        GPIO_PortClear(GPIOC, 1U << OLED_SCL_PIN); delay_us(5);
        GPIO_PortSet(GPIOC,   1U << OLED_SCL_PIN); delay_us(5);
    }
    PORT_SetPinConfig(OLED_SCL_PORT, OLED_SCL_PIN, &k_i2c_pin); delay_ms(10);
    CLOCK_EnableClock(OLED_I2C_CLK);
    i2c_master_config_t cfg; I2C_MasterGetDefaultConfig(&cfg);
    cfg.baudRate_Bps = 100000U;
    I2C_MasterInit(OLED_I2C, &cfg, CLOCK_GetFreq(kCLOCK_BusClk)); delay_ms(50);
    g_oled_ok = I2C_Ping(OLED_I2C, g_oled_addr);
    if (g_oled_ok) { oled_hw_init(); delay_ms(50); for (uint8_t p = 0; p < 8; p++) oled_clear_page(p); }
}

/*
 * OLED Initialization
 * Tries three baudrates (100k, 50k, 10k) and both possible I2C addresses.
 * Between attempts, runs the 9-pulse SCL recovery sequence to handle a
 * stuck bus from a previous failed init. This was the key fix for the
 * Windows-specific DWT timing issue: DWT_Init() must already be called
 * before this function executes, or all delay_ms() calls resolve to zero.
 */
static void OLED_Init(void) {
    LED_On(); CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(OLED_SCL_PORT, OLED_SCL_PIN, &k_i2c_pin);
    PORT_SetPinConfig(OLED_SDA_PORT, OLED_SDA_PIN, &k_i2c_pin);
    delay_ms(I2C_SETTLE_MS); CLOCK_EnableClock(OLED_I2C_CLK); delay_ms(10);
    const uint32_t bauds[] = {100000U, 50000U, 10000U};
    bool found = false;
    for (int attempt = 0; attempt < 3 && !found; attempt++) {
        i2c_master_config_t cfg; I2C_MasterGetDefaultConfig(&cfg);
        cfg.baudRate_Bps = bauds[attempt];
        I2C_MasterInit(OLED_I2C, &cfg, CLOCK_GetFreq(kCLOCK_BusClk)); delay_ms(50);
        if      (I2C_Ping(OLED_I2C, OLED_ADDR_3C)) { g_oled_addr = OLED_ADDR_3C; found = true; LED_Blink(2, 100); }
        else if (I2C_Ping(OLED_I2C, OLED_ADDR_3D)) { g_oled_addr = OLED_ADDR_3D; found = true; LED_Blink(3, 100); }
        if (!found) {
            /* Attempt failed: de-init and run SCL recovery before retrying */
            I2C_MasterDeinit(OLED_I2C); delay_ms(10);
            PORT_SetPinMux(OLED_SCL_PORT, OLED_SCL_PIN, kPORT_MuxAsGpio);
            gpio_pin_config_t scl_out = {kGPIO_DigitalOutput, 1};
            GPIO_PinInit(GPIOC, OLED_SCL_PIN, &scl_out);
            for (int p = 0; p < 9; p++) {
                GPIO_PortClear(GPIOC, 1U << OLED_SCL_PIN); delay_us(5);
                GPIO_PortSet(GPIOC,   1U << OLED_SCL_PIN); delay_us(5);
            }
            PORT_SetPinConfig(OLED_SCL_PORT, OLED_SCL_PIN, &k_i2c_pin); delay_ms(10);
        }
    }
    if (!found) { g_oled_ok = false; LED_Off(); return; }
    g_oled_ok = true; delay_ms(I2C_SETTLE_MS);
    oled_hw_init(); delay_ms(100);
    for (uint8_t p = 0; p < 8; p++) oled_clear_page(p);
    oled_print(0, 0, "SYSTEM INIT"); oled_print(2, 0, "READY"); LED_Off();
}

/* Update OLED display — rate-limited to once per 100ms to reduce I2C traffic */
static void OLED_Update(uint8_t temp, uint8_t hum, bool dht_ok,
                        bool mq2_ok, const char *smoke,
                        bool mq135_ok, const char *aq,
                        bool flame_present, bool flame,
                        bool mlx_ok, int16_t ir,
                        bool radar) {
    if (!g_oled_ok) return;
    static uint32_t last_ms = 0; uint32_t now = millis();
    if (now - last_ms < 100U) return; last_ms = now;
    char ln[24];

    /* Page 0: Temperature and Humidity from DHT11 */
    oled_clear_page(0);
    if (dht_ok) snprintf(ln, sizeof(ln), "T:%02uC H:%02u%%    ", temp, hum);
    else        snprintf(ln, sizeof(ln), "T:-- H:--        ");
    oled_print(0, 0, ln);

    /* Page 2: IR temperature (MLX90614) and Air Quality (MQ-135) */
    oled_clear_page(2);
    if (!mlx_ok) snprintf(ln, sizeof(ln), "IR:-- AQ:%s      ", mq135_ok ? aq : "ND");
    else {
        int16_t iw = ir / 10, ifr = ir % 10; if (ifr < 0) ifr = -ifr;
        snprintf(ln, sizeof(ln), "IR:%d.%dC %s      ", iw, ifr, mq135_ok ? aq : "ND");
    }
    oled_print(2, 0, ln);

    /* Page 4: Smoke level (MQ-2) and Flame detection (KY-026) */
    oled_clear_page(4);
    if      (!mq2_ok && !flame_present) snprintf(ln, sizeof(ln), "S:ND F:NO SEN    ");
    else if (!mq2_ok)                   snprintf(ln, sizeof(ln), "S:ND F:%s       ", flame ? "DET" : "NONE");
    else if (!flame_present)            snprintf(ln, sizeof(ln), "S:%s F:NO SEN    ", smoke);
    else                                snprintf(ln, sizeof(ln), "S:%s F:%s        ", smoke, flame ? "DET" : "NO");
    oled_print(4, 0, ln);

    /* Page 6: RCWL-0516 radar presence status */
    oled_clear_page(6);
    snprintf(ln, sizeof(ln), "RADAR:%s         ", radar ? "YES" : "NO");
    oled_print(6, 0, ln);

    LED_Toggle(); /* Heartbeat: LED toggles on each display update */
}

/* =============================================================================
   Section 16: UART1 – ESP8266 Communication -- Created with help of AI
   JSON telemetry is sent once per second. The ESP8266 parses each field using
   its own parseJsonInt() / parseJsonStr() helpers. All new fields (RADAR, AX,
   AY, AZ) are appended without requiring changes to the ESP8266 parser for
   backwards-compatible fields.
   ============================================================================= */

static void UART_Init_WiFi(void) {
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(WIFI_UART_PORT, WIFI_UART_TX_PIN, kPORT_MuxAlt3); /* UART1 TX */
    PORT_SetPinMux(WIFI_UART_PORT, WIFI_UART_RX_PIN, kPORT_MuxAlt3); /* UART1 RX */
    CLOCK_EnableClock(WIFI_UART_CLK);
    uart_config_t cfg; UART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = WIFI_UART_BAUD; cfg.enableTx = true; cfg.enableRx = true;
    UART_Init(WIFI_UART, &cfg, 180000000U); delay_ms(100);
}

/*
 * Build and transmit JSON telemetry to ESP8266.
 * Format: {"T":n,"H":n,"S":n,"F":n,"IR":n,"AQ":"str",
 *           "AX":n,"AY":n,"AZ":n,"RADAR":n,"ts":n}
 * RADAR: 1 = presence detected, 0 = clear
 * Accelerometer values in milli-g (1000 = 1.000g)
 */
static void UART_SendJSON(uint8_t temp, uint8_t hum, bool dht_ok,
                          bool mq2_ok, uint8_t smoke_code,
                          bool mq135_ok, const char *aq,
                          bool flame_present, bool flame,
                          bool mlx_ok, int16_t ir,
                          bool radar, uint32_t uptime_s) {
    /* Apply Z-axis calibration offset before converting to milli-g */
    int16_t ax_mg = g_accel_ok ? raw_to_mg(g_ax_raw) : 0;
    int16_t ay_mg = g_accel_ok ? raw_to_mg(g_ay_raw) : 0;
    int16_t az_mg = g_accel_ok ? raw_to_mg((int16_t)(g_az_raw - g_az_offset)) : 0;
    char buf[240];
    int n = snprintf(buf, sizeof(buf),
        "{\"T\":%d,\"H\":%d,\"S\":%u,\"F\":%u,\"IR\":%d,"
        "\"AQ\":\"%s\","
        "\"AX\":%d,\"AY\":%d,\"AZ\":%d,"
        "\"RADAR\":%u,"
        "\"ts\":%lu}\r\n",
        dht_ok ? (int)temp : -1,          /* -1 = sensor offline             */
        dht_ok ? (int)hum  : -1,
        mq2_ok ? (unsigned)smoke_code : 0U,
        (flame_present && flame) ? 1U : 0U,
        mlx_ok ? (int)ir : (int)INT16_MIN,
        mq135_ok ? aq : "ND",             /* "ND" = not detected/available   */
        (int)ax_mg, (int)ay_mg, (int)az_mg,
        radar ? 1U : 0U,
        (unsigned long)uptime_s);
    if (n > 0) UART_WriteBlocking(WIFI_UART, (uint8_t *)buf, (size_t)n);
}

/* =============================================================================
   Section 17: Main Entry Point
   Boot sequence:
     1. DWT_Init (MUST be first — required for all delay_ms calls)
     2. Board clocks and GPIO
     3. OLED init (halts on failure — OLED is required for user feedback)
     4. I2C0 init (tristated pins) -> MLX90614 -> FXOS8700CQ
     5. UART1 -> ESP8266
     6. Main sensor loop (1 Hz sample + 20 Hz accelerometer + continuous buzzer)
   ============================================================================= */

int main(void) {
    /* DWT MUST be initialized first — see Section 4 comments */
    DWT_Init(); delay_ms(INIT_DELAY_MS);
    BOARD_InitBootPins(); BOARD_InitBootClocks();
    DWT_Init(); /* Re-init after clock reconfiguration to reset cycle counter */
    LED_Init(); LED_Blink(1, 500); /* Single long blink = boot started        */
    InitGPIO();
    ADC_Init_Local();

    delay_ms(INIT_DELAY_MS);
    OLED_Init();
    /* OLED is required for status feedback — halt if not found */
    if (!g_oled_ok) { for (;;) { LED_Blink(2, 150); delay_ms(500); } }

    oled_print(0, 0, "OLED OK");
    oled_print(2, 0, "INIT I2C0");
    delay_ms(400);

    /* Tristate all I2C0 pins before peripheral init to prevent bus fight.
       The first I2C0_SelectMLX/FXOS call activates the correct pair. */
    CLOCK_EnableClock(kCLOCK_PortB); CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinConfig(MLX_SCL_PORT,  MLX_SCL_PIN,  &k_tristate_pin);
    PORT_SetPinConfig(MLX_SDA_PORT,  MLX_SDA_PIN,  &k_tristate_pin);
    PORT_SetPinConfig(FXOS_SCL_PORT, FXOS_SCL_PIN, &k_tristate_pin);
    PORT_SetPinConfig(FXOS_SDA_PORT, FXOS_SDA_PIN, &k_tristate_pin);
    SharedI2C_PeripheralInit();

    oled_clear_page(2); oled_print(2, 0, "INIT MLX");
    MLX90614_Init();
    oled_clear_page(2); oled_print(2, 0, g_mlx_ok ? "MLX OK" : "MLX FAIL");
    delay_ms(400);

    oled_clear_page(2); oled_print(2, 0, "INIT ACCEL");
    FXOS_Init();
    oled_clear_page(2); oled_print(2, 0, g_accel_ok ? "ACCEL OK" : "ACCEL FAIL");
    delay_ms(400);

    UART_Init_WiFi(); delay_ms(500);

    /* Display final boot status on OLED */
    for (uint8_t p = 0; p < 8; p++) oled_clear_page(p);
    oled_print(0, 0, "ALL SYSTEMS");
    oled_print(2, 0, g_accel_ok ? "ACCEL:YES" : "ACCEL:NO");
    oled_print(4, 0, g_mlx_ok   ? "MLX:YES"   : "MLX:NO");
    oled_print(6, 0, "RUNNING");
    delay_ms(1000); LED_Blink(3, 100); /* 3 blinks = ready                   */

    /* ── Runtime State Variables ── */
    typedef enum { S_NRM=0, S_WRN=1, S_DNG=2 } smoke_lvl_t;
    smoke_lvl_t smoke_lvl = S_NRM;
    uint8_t  last_temp = 0, last_hum = 0;
    bool     last_dht_ok = false;
    uint16_t hum_f4 = 0; bool hum_init = false; /* Fixed-point humidity EMA */
    bool temp_warn = false, hum_warn = false, pump_on = false, fan_on = false;
    bool flame = false, flame_present = false, mq2_ok = false, mq135_ok = false;
    int16_t    ir_temp   = INT16_MIN;
    const char *aq_label = "GOOD";
    uint32_t last_sample_ms  = millis();
    uint32_t last_heartbeat  = millis();
    uint32_t last_accel_ms   = millis();
    uint32_t last_oled_chk   = millis();
    uint32_t uptime_s        = 0;

    /* ── Main Loop ── */
    for (;;) {
        Buzzer_Tick(); /* Must run every iteration for accurate beep timing    */
        uint32_t now = millis();

        /* Heartbeat LED: 1 blink every 2 seconds = system alive */
        if (now - last_heartbeat >= 2000U) { last_heartbeat = now; LED_Blink(1, 50); }

        /* OLED watchdog: attempt bus recovery if OLED stops responding */
        if (now - last_oled_chk >= OLED_WATCHDOG_MS) {
            last_oled_chk = now;
            if (!g_oled_ok || !I2C_Ping(OLED_I2C, g_oled_addr)) OLED_BusRecover();
        }

        /* Accelerometer at 20 Hz — mux switches to PTD8/PTD9 inside */
        if (now - last_accel_ms >= 50U) { last_accel_ms = now; FXOS_ReadXYZ_Averaged(); }

        /* 1 Hz sensor sample and telemetry cycle */
        if (now - last_sample_ms >= 1000U) {
            last_sample_ms = now; uptime_s++;

            /* Check which sensor modules are physically connected */
            mq2_ok        = MQ2_Present();
            mq135_ok      = MQ135_Present();
            flame_present = Flame_Present();

            /* RCWL-0516: direct GPIO read, no protocol — lowest latency */
            g_radar = Radar_Detected();

            /* MLX IR thermometer — mux switches to PTB2/PTB3 inside each call */
            MLX90614_CheckConnection(uptime_s);
            ir_temp = MLX90614_ReadTemp();

            /* Flame: confirm via ADC reading only if DO indicates presence */
            if (flame_present) {
                uint32_t fmv = ADC_ReadMV(FLAME_AO_CH);
                flame = (fmv < FLAME_DETECT_MV); /* Low voltage = flame nearby */
            } else { flame = false; }

            /* Smoke level: hysteresis state machine prevents rapid transitions */
            if (mq2_ok) {
                uint32_t smv = ADC_ReadMV(MQ2_AO_CH);
                switch (smoke_lvl) {
                    case S_NRM:
                        if      (smv >= SMOKE_DANGER_MV) smoke_lvl = S_DNG;
                        else if (smv >= SMOKE_WARN_MV)   smoke_lvl = S_WRN; break;
                    case S_WRN:
                        if      (smv >= SMOKE_DANGER_MV)               smoke_lvl = S_DNG;
                        else if (smv <  SMOKE_WARN_MV - SMOKE_HYST_MV) smoke_lvl = S_NRM; break;
                    case S_DNG:
                        if (smv < SMOKE_DANGER_MV - SMOKE_HYST_MV)
                            smoke_lvl = (smv >= SMOKE_WARN_MV) ? S_WRN : S_NRM; break;
                }
            } else { smoke_lvl = S_NRM; }

            /* Air quality from MQ-135 */
            aq_label = mq135_ok ? AQ_Label(ADC_ReadMV(MQ135_AO_CH)) : "ND";

            /*
             * DHT11 dual-read strategy: take two readings 60ms apart,
             * prefer the second. This eliminates checksum failures caused
             * by bus capacitance slowing edges on longer wire runs.
             */
            uint8_t t1=0,h1=0,t2=0,h2=0; bool ok=false;
            bool r1 = DHT11_Read(&t1, &h1); delay_ms(60);
            bool r2 = DHT11_Read(&t2, &h2);
            if      (r2) { last_temp = t2; last_hum = h2; ok = true; }
            else if (r1) { last_temp = t1; last_hum = h1; ok = true; }
            last_dht_ok = ok;

            uint8_t hum_show = last_hum;
            if (ok) {
                /* Exponential moving average on humidity to reduce jitter
                   Formula: hum_f4 = (3/4 * hum_f4) + (1/4 * new_reading)
                   Fixed-point: multiply values by 4 to avoid floats       */
                if (!hum_init) { hum_f4 = (uint16_t)last_hum * 4; hum_init = true; }
                else            hum_f4 = (uint16_t)((3U * hum_f4 + (uint16_t)last_hum * 4U) / 4U);
                hum_show = (uint8_t)(hum_f4 / 4U);

                /* Temperature and humidity warning flags with hysteresis */
                if (!temp_warn) { if (last_temp >= TEMP_WARN_C)               temp_warn = true;  }
                else            { if (last_temp <= TEMP_WARN_C - TEMP_HYST_C)  temp_warn = false; }
                if (!hum_warn)  { if (hum_show  >= HUM_WARN_PCT)               hum_warn  = true;  }
                else            { if (hum_show  <= HUM_WARN_PCT - HUM_HYST_PCT) hum_warn = false; }

                /* Pump: activates on high temperature OR confirmed flame */
                if (!pump_on) { if (last_temp >= PUMP_ON_TEMP_C || (flame_present && flame))  { pump_on = true;  Pump_Set(true);  } }
                else          { if (last_temp <= PUMP_OFF_TEMP_C && !(flame_present && flame)) { pump_on = false; Pump_Set(false); } }

                /* Fan: activates on high temperature */
                if (!fan_on) { if (last_temp >= FAN_ON_TEMP_C)  { fan_on = true;  Fan_Set(true);  } }
                else         { if (last_temp <= FAN_OFF_TEMP_C) { fan_on = false; Fan_Set(false); } }
            } else {
                /* DHT11 read failed: safe state — turn off actuators */
                temp_warn = hum_warn = pump_on = fan_on = false;
                Pump_Set(false); Fan_Set(false);
            }

            const char *smoke_txt = (smoke_lvl == S_DNG) ? "DNG"
                                  : (smoke_lvl == S_WRN) ? "WRN" : "NRM";

            /* Update OLED and send JSON telemetry */
            OLED_Update(last_temp, hum_show, last_dht_ok,
                        mq2_ok, smoke_txt, mq135_ok, aq_label,
                        flame_present, flame, g_mlx_ok, ir_temp,
                        g_radar);

            UART_SendJSON(last_temp, hum_show, last_dht_ok,
                          mq2_ok, (uint8_t)smoke_lvl, mq135_ok, aq_label,
                          flame_present, flame, g_mlx_ok, ir_temp,
                          g_radar, uptime_s);

            /*
             * Buzzer priority (highest to lowest):
             *   SOLID  – flame confirmed, smoke DANGER, or extreme temp
             *   BEEP   – smoke WARNING, or temp + humidity warning
             *   OFF    – all clear
             * RADAR detection is logged to dashboard only; not audible,
             * since false positives would alarm vulnerable patients.
             */
            bool danger = (flame_present && flame)
                       || (mq2_ok && smoke_lvl == S_DNG)
                       || (last_dht_ok && last_temp >= TEMP_DANGER_C);
            bool warn = !danger
                     && ((mq2_ok && smoke_lvl == S_WRN)
                         || (last_dht_ok && (temp_warn || hum_warn)));

            if      (danger) Buzzer_SetMode(BUZ_SOLID);
            else if (warn)   Buzzer_SetMode(BUZ_BEEP);
            else             Buzzer_SetMode(BUZ_OFF);
        }

        delay_ms(5); /* Yield to watchdog / avoid runaway loop */
    }
}