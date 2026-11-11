/*
 * Day 12 — Pocket Programmable 1.0V–15.0V SEPIC Micro Bench Supply & Electronic Fuse
 * Target: STM32F072CBT6 (Cortex-M0 @ 48 MHz Crystal-Less HSI48 + CRS USB SOF Trim)
 *
 * Pin Mapping (matches Day_12_Bench_Power_Supply.kicad_sch):
 *   PA0  : EFUSE_ALERT  (EXTI0 Falling-Edge IRQ from U6 INA226 ~Alert, 140 us SOL hardware trip)
 *   PA1  : VBUS_SENSE   (ADC_IN1 10k/10k host bay VBUS droop monitor)
 *   PA11 : USB_DM       (USB 2.0 FS CDC-ACM SCPI/CLI bench control channel)
 *   PA12 : USB_DP
 *   PB0  : PWR_EN       (TIM3_CH3 AF1 25 kHz PWM -> Q2 AO3400A -> Q1 AO3401A pulsed soft-start)
 *   PB1  : SEPIC_EN     (MT3608 enable; pulled low by R14 47k during boot/fault for 0.00V DC isolation)
 *   PB6  : I2C1_SCL     (AF1, 400 kHz Fast-Mode I2C1 bus -> MCP4725 0x60 & INA226 0x40)
 *   PB7  : I2C1_SDA     (AF1, 400 kHz Fast-Mode I2C1 bus)
 *   PB12 : OUT_LED      (Green 0603 Output-Enabled LED D4)
 *   PB13 : FAULT_LED    (Red 0603 E-Fuse OCP / VBUS UVLO Fault LED D5)
 */

#include <stdint.h>

#define PERIPH_BASE     0x40000000UL
#define AHB2_BASE       0x48000000UL

#define TIM3_BASE       (PERIPH_BASE + 0x00000400UL)
#define I2C1_BASE       (PERIPH_BASE + 0x00005400UL)
#define USB_BASE        (PERIPH_BASE + 0x00005C00UL)
#define CRS_BASE        (PERIPH_BASE + 0x00006C00UL)
#define SYSCFG_BASE     (PERIPH_BASE + 0x00010000UL)
#define EXTI_BASE       (PERIPH_BASE + 0x00010400UL)
#define ADC1_BASE       (PERIPH_BASE + 0x00012400UL)
#define RCC_BASE        (PERIPH_BASE + 0x00021000UL)
#define GPIOA_BASE      (AHB2_BASE   + 0x00000000UL)
#define GPIOB_BASE      (AHB2_BASE   + 0x00000400UL)
#define NVIC_ISER       (*(volatile uint32_t *)0xE000E100UL)

#define REG32(addr)     (*(volatile uint32_t *)(addr))

/* RCC, CRS & SYSCFG */
#define RCC_CFGR        REG32(RCC_BASE + 0x04UL)
#define RCC_AHBENR      REG32(RCC_BASE + 0x14UL)
#define RCC_APB2ENR     REG32(RCC_BASE + 0x18UL)
#define RCC_APB1ENR     REG32(RCC_BASE + 0x1CUL)
#define RCC_CR2         REG32(RCC_BASE + 0x34UL)
#define CRS_CR          REG32(CRS_BASE + 0x00UL)
#define SYSCFG_EXTICR1  REG32(SYSCFG_BASE + 0x08UL)

/* EXTI */
#define EXTI_IMR        REG32(EXTI_BASE + 0x00UL)
#define EXTI_FTSR       REG32(EXTI_BASE + 0x0CUL)
#define EXTI_PR         REG32(EXTI_BASE + 0x14UL)

/* GPIO */
#define GPIOA_MODER     REG32(GPIOA_BASE + 0x00UL)
#define GPIOB_MODER     REG32(GPIOB_BASE + 0x00UL)
#define GPIOB_OTYPER    REG32(GPIOB_BASE + 0x04UL)
#define GPIOB_BSRR      REG32(GPIOB_BASE + 0x18UL)
#define GPIOB_AFRL      REG32(GPIOB_BASE + 0x20UL)

/* TIM3 (25 kHz PWM on CH3 = PB0 for pulsed soft-start of Q1 AO3401A) */
#define TIM3_CR1        REG32(TIM3_BASE + 0x00UL)
#define TIM3_CCMR2      REG32(TIM3_BASE + 0x1CUL)
#define TIM3_CCER       REG32(TIM3_BASE + 0x20UL)
#define TIM3_PSC        REG32(TIM3_BASE + 0x28UL)
#define TIM3_ARR        REG32(TIM3_BASE + 0x2CUL)
#define TIM3_CCR3       REG32(TIM3_BASE + 0x3CUL)

/* ADC1 (Host bay VBUS droop monitor on PA1 = ADC_IN1) */
#define ADC1_ISR        REG32(ADC1_BASE + 0x00UL)
#define ADC1_CR         REG32(ADC1_BASE + 0x08UL)
#define ADC1_CHSELR     REG32(ADC1_BASE + 0x28UL)
#define ADC1_DR         REG32(ADC1_BASE + 0x40UL)

/* I2C1 */
#define I2C1_CR1        REG32(I2C1_BASE + 0x00UL)
#define I2C1_CR2        REG32(I2C1_BASE + 0x04UL)
#define I2C1_TIMINGR    REG32(I2C1_BASE + 0x10UL)
#define I2C1_ISR        REG32(I2C1_BASE + 0x18UL)
#define I2C1_ICR        REG32(I2C1_BASE + 0x1CUL)
#define I2C1_RXDR       REG32(I2C1_BASE + 0x24UL)
#define I2C1_TXDR       REG32(I2C1_BASE + 0x28UL)

/* USB */
#define USB_CNTR        REG32(USB_BASE + 0x40UL)

#define MCP4725_ADDR    0x60U
#define INA226_ADDR     0x40U

/*
 * Feedback Summing Math (R16 = 100k, R17 = 5.1k, R18 = 22k, V_FB = 0.600 V):
 *   V_OUT(mV) = 15092 - (V_DAC_mV * 100) / 22
 *   dac_code  = ((15092 - target_mv) * 4095 * 22) / (3300 * 100)
 */
#define VOUT_MIN_MV     1000U
#define VOUT_MAX_MV     15000U
#define MAX_POWER_MW    4000U   /* Clamps 5V VBUS input draw < 0.95 A (safe on 1.5A & 3.0A ports) */
#define VBUS_UVLO_MV    4550U

static volatile uint8_t  efuse_fault_latched;
static volatile uint8_t  output_enabled;
static uint16_t          target_vout_mv = 5000U;
static uint16_t          ilim_ma        = 500U;
static uint16_t          current_dac    = 2746U;

static void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < (us * 8UL); i++) {
        __asm volatile ("nop");
    }
}

/* Sub-microsecond hardware electronic-fuse trip ISR (INA226 140 us SOL alert on PA0) */
void EXTI0_1_IRQHandler(void) {
    if ((EXTI_PR & 1UL) != 0UL) {
        /* 1. Kill MT3608 SEPIC switching immediately (PB1 LOW) & open Q1 AO3401A (TIM3 CCR3 = 0) */
        GPIOB_BSRR = (1UL << (1U + 16U)) | (1UL << (12U + 16U)) | (1UL << 13U);
        TIM3_CCR3  = 0UL;
        output_enabled      = 0U;
        efuse_fault_latched = 1U;
        EXTI_PR             = 1UL;
    }
}

static void clock_and_gpio_init(void) {
    /* Enable 48 MHz HSI48 + 14 MHz HSI14 ADC oscillator */
    RCC_CR2 |= (1UL << 16) | (1UL << 0);
    while ((RCC_CR2 & (1UL << 17)) == 0UL) {
    }
    RCC_CFGR = (RCC_CFGR & ~0x3UL) | 0x3UL;
    while (((RCC_CFGR >> 2) & 0x3UL) != 0x3UL) {
    }

    /* Clocks: GPIOA, GPIOB; SYSCFG + ADC1; TIM3 + I2C1 + USB + CRS */
    RCC_AHBENR  |= (1UL << 17) | (1UL << 18);
    RCC_APB2ENR |= (1UL << 0)  | (1UL << 9);
    RCC_APB1ENR |= (1UL << 1)  | (1UL << 21) | (1UL << 23) | (1UL << 27);
    CRS_CR      |= (1UL << 6)  | (1UL << 5);

    /* Safe OFF state before configuring outputs: SEPIC_EN=0, OUT_LED=0, FAULT_LED=0 */
    GPIOB_BSRR = (1UL << (1U + 16U)) | (1UL << (12U + 16U)) | (1UL << (13U + 16U));

    /* PA0 = Input (EFUSE_ALERT), PA1 = Analog (VBUS_SENSE) */
    GPIOA_MODER = (GPIOA_MODER & ~0xFUL) | (0x3UL << 2);

    /* PB0 = AF1 (TIM3_CH3), PB1 = Output, PB6/PB7 = AF1 Open-Drain (I2C1), PB12/PB13 = Output */
    GPIOB_OTYPER |= (1UL << 6) | (1UL << 7);
    GPIOB_AFRL    = (GPIOB_AFRL & ~((0xFUL << 0) | (0xFFUL << 24)))
                  | (1UL << 0) | (1UL << 24) | (1UL << 28);
    GPIOB_MODER   = (GPIOB_MODER & ~((0xFUL << 0) | (0xFUL << 12) | (0xFUL << 24)))
                  | (0x2UL << 0) | (0x1UL << 2) | (0xAUL << 12) | (0x5UL << 24);

    /* TIM3_CH3 @ 25 kHz PWM (48 MHz / 1920 = 25 kHz) for pulsed Q1 soft-start */
    TIM3_PSC   = 0UL;
    TIM3_ARR   = 1919UL;
    TIM3_CCR3  = 0UL;
    TIM3_CCMR2 = (6UL << 4) | (1UL << 3);
    TIM3_CCER  = (1UL << 8);
    TIM3_CR1   = 1UL;

    /* I2C1 @ 400 kHz Fast-Mode (48 MHz SYSCLK) */
    I2C1_TIMINGR = 0x00901850UL;
    I2C1_CR1     = 1UL;

    /* ADC1 calibration & enable on CH1 (PA1 VBUS_SENSE) */
    if ((ADC1_CR & 1UL) == 0UL) {
        ADC1_CR |= (1UL << 31);
        while ((ADC1_CR & (1UL << 31)) != 0UL) {
        }
        ADC1_CR |= 1UL;
    }
    ADC1_CHSELR = (1UL << 1);

    /* EXTI0 falling-edge interrupt on PA0 (INA226 ~Alert) */
    SYSCFG_EXTICR1 &= ~0xFUL;
    EXTI_FTSR      |= 1UL;
    EXTI_PR         = 1UL;
    EXTI_IMR       |= 1UL;
    NVIC_ISER       = (1UL << 5);

    /* Release USB FS transceiver power-down */
    USB_CNTR = 0UL;
}

static void i2c_write_bytes(uint8_t addr7, const uint8_t *buf, uint8_t len) {
    I2C1_CR2 = ((uint32_t)addr7 << 1) | ((uint32_t)len << 16) | (1UL << 25) | (1UL << 13);
    for (uint8_t i = 0; i < len; i++) {
        uint32_t guard = 20000UL;
        while (((I2C1_ISR & (1UL << 1)) == 0UL) && (--guard > 0UL)) {
        }
        I2C1_TXDR = buf[i];
    }
    uint32_t guard = 20000UL;
    while (((I2C1_ISR & (1UL << 5)) == 0UL) && (--guard > 0UL)) {
    }
    I2C1_ICR = (1UL << 5);
}

static uint16_t ina226_read_reg(uint8_t reg) {
    I2C1_CR2 = ((uint32_t)INA226_ADDR << 1) | (1UL << 16) | (1UL << 13);
    uint32_t guard = 20000UL;
    while (((I2C1_ISR & (1UL << 1)) == 0UL) && (--guard > 0UL)) {
    }
    I2C1_TXDR = reg;
    guard = 20000UL;
    while (((I2C1_ISR & (1UL << 6)) == 0UL) && (--guard > 0UL)) {
    }

    I2C1_CR2 = ((uint32_t)INA226_ADDR << 1) | (1UL << 10) | (2UL << 16) | (1UL << 25) | (1UL << 13);
    uint16_t val = 0U;
    for (uint8_t i = 0; i < 2U; i++) {
        guard = 20000UL;
        while (((I2C1_ISR & (1UL << 2)) == 0UL) && (--guard > 0UL)) {
        }
        val = (uint16_t)((val << 8) | (uint8_t)I2C1_RXDR);
    }
    I2C1_ICR = (1UL << 5);
    return val;
}

static void mcp4725_set_code(uint16_t code12) {
    if (code12 > 4095U) {
        code12 = 4095U;
    }
    current_dac = code12;
    uint8_t pkt[2];
    pkt[0] = (uint8_t)((code12 >> 8) & 0x0FU);
    pkt[1] = (uint8_t)(code12 & 0xFFU);
    i2c_write_bytes(MCP4725_ADDR, pkt, 2U);
}

static uint16_t vout_mv_to_dac(uint16_t mv) {
    if (mv < VOUT_MIN_MV) {
        mv = VOUT_MIN_MV;
    }
    if (mv > VOUT_MAX_MV) {
        mv = VOUT_MAX_MV;
    }
    uint32_t span = 15092UL - (uint32_t)mv;
    uint32_t code = (span * 4095UL * 22UL) / (3300UL * 100UL);
    return (code > 4095UL) ? 4095U : (uint16_t)code;
}

static void ina226_configure_efuse(uint16_t limit_ma) {
    /*
     * 140 us shunt + 140 us bus continuous conversion (AVG=1 -> 0.14 ms hardware trip!)
     * R_shunt = 50 mOhm -> V_shunt_LSB = 2.5 uV -> 1 mA = 50 uV = 20 LSBs!
     */
    static const uint8_t cfg_pkt[3] = { 0x00U, 0x40U, 0x07U };
    i2c_write_bytes(INA226_ADDR, cfg_pkt, 3U);

    uint16_t sol_ticks = (uint16_t)(limit_ma * 20U);
    uint8_t lim_pkt[3];
    lim_pkt[0] = 0x07U;
    lim_pkt[1] = (uint8_t)(sol_ticks >> 8);
    lim_pkt[2] = (uint8_t)(sol_ticks & 0xFFU);
    i2c_write_bytes(INA226_ADDR, lim_pkt, 3U);

    /* Mask/Enable: SOL (bit 15 = 1) + Alert Latch Enable (bit 0 = 1) */
    static const uint8_t msk_pkt[3] = { 0x06U, 0x80U, 0x01U };
    i2c_write_bytes(INA226_ADDR, msk_pkt, 3U);
}

static uint16_t read_host_vbus_mv(void) {
    ADC1_CR |= (1UL << 2);
    uint32_t guard = 10000UL;
    while (((ADC1_ISR & (1UL << 2)) == 0UL) && (--guard > 0UL)) {
    }
    uint32_t raw = ADC1_DR & 0xFFFUL;
    /* 10k / 10k divider -> VBUS_mV = raw * 6600 / 4095 */
    return (uint16_t)((raw * 6600UL) / 4095UL);
}

static void bench_enable_output(uint16_t req_mv, uint16_t req_ma) {
    /* Enforce 4.0 W continuous output power envelope so 5V VBUS input stays < 0.95 A */
    uint32_t max_ma_for_power = (MAX_POWER_MW * 1000UL) / (uint32_t)req_mv;
    if ((uint32_t)req_ma > max_ma_for_power) {
        req_ma = (uint16_t)max_ma_for_power;
    }

    target_vout_mv      = req_mv;
    ilim_ma             = req_ma;
    efuse_fault_latched = 0U;

    /* 1. Pre-bias DAC to lowest rail (1.00 V) before closing input switch */
    mcp4725_set_code(vout_mv_to_dac(VOUT_MIN_MV));
    ina226_configure_efuse(ilim_ma);
    (void)ina226_read_reg(0x06U); /* Clear any prior latched alert */

    /* 2. 12 ms 25 kHz PWM pulsed precharge of C7 (10 uF) via Q1 AO3401A (< 0.15 A inrush) */
    GPIOB_BSRR = (1UL << (13U + 16U));
    for (uint32_t step = 1UL; step <= 24UL; step++) {
        TIM3_CCR3 = (step * 1919UL) / 24UL;
        delay_us(500UL);
        if (read_host_vbus_mv() < VBUS_UVLO_MV) {
            TIM3_CCR3  = 0UL;
            GPIOB_BSRR = (1UL << 13U);
            return;
        }
    }

    /* 3. Enable MT3608 SEPIC stage (PB1 HIGH) and slew DAC smoothly to target_vout_mv */
    GPIOB_BSRR     = (1UL << 1U) | (1UL << 12U);
    output_enabled = 1U;

    uint16_t goal_dac = vout_mv_to_dac(target_vout_mv);
    while ((output_enabled != 0U) && (current_dac != goal_dac)) {
        if (current_dac > goal_dac) {
            uint16_t step = (uint16_t)(current_dac - goal_dac);
            mcp4725_set_code((uint16_t)(current_dac - ((step > 16U) ? 16U : step)));
        } else {
            uint16_t step = (uint16_t)(goal_dac - current_dac);
            mcp4725_set_code((uint16_t)(current_dac + ((step > 16U) ? 16U : step)));
        }
        delay_us(250UL);
    }
}

int main(void) {
    clock_and_gpio_init();

    /* Boot safe at 0.00 V output (Q1 open, SEPIC_EN=0, DAC parked at 1.0 V floor) */
    mcp4725_set_code(vout_mv_to_dac(VOUT_MIN_MV));
    ina226_configure_efuse(ilim_ma);

    for (;;) {
        if (output_enabled != 0U) {
            /* Monitor host bay VBUS UVLO & closed-loop INA226 1.25 mV fine voltage trim */
            if (read_host_vbus_mv() < VBUS_UVLO_MV) {
                GPIOB_BSRR          = (1UL << (1U + 16U)) | (1UL << (12U + 16U)) | (1UL << 13U);
                TIM3_CCR3           = 0UL;
                output_enabled      = 0U;
                efuse_fault_latched = 1U;
            } else {
                uint32_t meas_mv = ((uint32_t)ina226_read_reg(0x02U) * 5UL) / 4UL;
                if ((meas_mv + 6UL < (uint32_t)target_vout_mv) && (current_dac > 0U)) {
                    mcp4725_set_code((uint16_t)(current_dac - 1U));
                } else if ((meas_mv > (uint32_t)target_vout_mv + 6UL) && (current_dac < 4095U)) {
                    mcp4725_set_code((uint16_t)(current_dac + 1U));
                }
            }
        } else if ((efuse_fault_latched == 0U) && (target_vout_mv == 0U)) {
            bench_enable_output(5000U, 500U);
        }
        delay_us(2000UL);
    }
}
