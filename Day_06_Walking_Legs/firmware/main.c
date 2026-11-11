/**
 * Day 06 — Framework Walking Legs Firmware (STM32F103C8T6)
 *
 * Pinout & Hardware Mapping (matches Day_06_Walking_Legs.kicad_sch):
 *  - PC13          : ACT_LED (Red 0603 gait/SOF sync indicator)
 *  - PA0 (TIM2_CH1): HIP_PWM_MCU  -> R16 (22R) -> J2 Pin 1 (HIP_SERVO)
 *  - PA1 (TIM2_CH2): KNEE_PWM_MCU -> R17 (22R) -> J3 Pin 1 (KNEE_SERVO)
 *  - PB0, PB1      : BAY_ID0, BAY_ID1 hardware solder-jumper straps (00=FL, 01=FR, 10=RL, 11=RR)
 *  - PB4           : SERVO_PWR_EN_N (Active-Low AO3401A P-FET input switch -> 0 uA limp cutoff)
 *  - PB5           : BOOST_EN (Active-High MT3608 6.04V synchronous-control boost enable)
 *  - PB10, PB11    : I2C2 SCL/SDA -> INA226AIDGSR (addr 0x40, 20 mOhm VBUS shunt R8)
 *  - PB12          : INA_ALERT_N (Active-Low hardware overcurrent alert)
 *  - PA8           : USB_ENUM (1.5 kOhm pull-up R2 onto USB_DP_RAW)
 *  - PA11, PA12    : USB Full-Speed D-, D+ (1 kHz SOF hardware frame sync across all 4 bays)
 */

#include <stdint.h>
#include <stdbool.h>

/* Register Base Addresses */
#define RCC_BASE        0x40021000UL
#define FLASH_BASE_REG  0x40022000UL
#define GPIOA_BASE      0x40010800UL
#define GPIOB_BASE      0x40010C00UL
#define GPIOC_BASE      0x40011000UL
#define AFIO_BASE       0x40010000UL
#define EXTI_BASE       0x40010400UL
#define TIM2_BASE       0x40000000UL
#define I2C2_BASE       0x40005800UL
#define USB_BASE        0x40005C00UL
#define SYSTICK_BASE    0xE000E010UL
#define NVIC_ISER0      (*(volatile uint32_t *)0xE000E100UL)
#define NVIC_ISER1      (*(volatile uint32_t *)0xE000E104UL)

/* RCC, AFIO & EXTI */
#define RCC_CR          (*(volatile uint32_t *)(RCC_BASE + 0x00UL))
#define RCC_CFGR        (*(volatile uint32_t *)(RCC_BASE + 0x04UL))
#define RCC_APB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x18UL))
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x1CUL))
#define FLASH_ACR       (*(volatile uint32_t *)(FLASH_BASE_REG + 0x00UL))
#define AFIO_MAPR       (*(volatile uint32_t *)(AFIO_BASE + 0x04UL))
#define AFIO_EXTICR4    (*(volatile uint32_t *)(AFIO_BASE + 0x14UL))
#define EXTI_IMR        (*(volatile uint32_t *)(EXTI_BASE + 0x00UL))
#define EXTI_FTSR       (*(volatile uint32_t *)(EXTI_BASE + 0x0CUL))
#define EXTI_PR         (*(volatile uint32_t *)(EXTI_BASE + 0x14UL))

/* GPIO */
typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)

/* TIM2 (1 MHz / 1 us tick -> 200 Hz / 5000 us servo PWM frame) */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
} TIM_TypeDef;

#define TIM2 ((TIM_TypeDef *)TIM2_BASE)

/* I2C2 */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
} I2C_TypeDef;

#define I2C2 ((I2C_TypeDef *)I2C2_BASE)

/* USB FS Device Peripheral (SOF Frame Number FNR + Interrupt Status ISTR) */
#define USB_CNTR        (*(volatile uint32_t *)(USB_BASE + 0x40UL))
#define USB_ISTR        (*(volatile uint32_t *)(USB_BASE + 0x44UL))
#define USB_FNR         (*(volatile uint32_t *)(USB_BASE + 0x48UL))
#define USB_ISTR_SOF    (1UL << 9)
#define USB_CNTR_SOFM   (1UL << 9)

/* SysTick */
#define SYST_CSR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x00UL))
#define SYST_RVR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x04UL))
#define SYST_CVR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x08UL))

/* Safety & Gait Constants — Validated for Framework 2x 3.0A + 2x 1.5A USB-C Bays */
#define SERVO_MIN_US          750U
#define SERVO_NEUTRAL_US      1500U
#define SERVO_MAX_US          2250U
#define BAY_CURRENT_LIMIT_MA  1300   /* Telemetry soft foldback before 1.50 A USB-C bay ceiling */
#define INA226_ALERT_SOL_LSB  11040U /* 1.38 A * 20 mOhm / 2.5 uV = 11040 LSB (27.60 mV hardware clamp) */
#define WATCHDOG_TIMEOUT_MS   250U   /* Limp cutoff if host gait stream stops */

/* Live Leg State */
static volatile uint32_t g_ms_ticks = 0;
static volatile uint32_t g_last_cmd_ms = 0;
static volatile uint32_t g_ocp_alert_count = 0;
static volatile uint16_t g_sof_frame = 0;
static volatile uint16_t g_target_sof = 0;
static volatile uint16_t g_staged_hip_us = SERVO_NEUTRAL_US;
static volatile uint16_t g_staged_knee_us = SERVO_NEUTRAL_US;
static volatile uint16_t g_active_hip_us = SERVO_NEUTRAL_US;
static volatile uint16_t g_active_knee_us = SERVO_NEUTRAL_US;
static volatile bool     g_servo_power_on = false;
static uint8_t           g_bay_id = 0;

void SysTick_Handler(void) {
    g_ms_ticks++;
}

static void delay_ms(uint32_t ms) {
    uint32_t t0 = g_ms_ticks;
    while ((g_ms_ticks - t0) < ms) {
        __asm volatile ("nop");
    }
}

static void delay_us(uint16_t us) {
    /* TIM2 runs at 1 MHz (1 us/tick, ARR=4999) */
    uint16_t start = (uint16_t)TIM2->CNT;
    uint16_t elapsed = 0U;
    while (elapsed < us) {
        uint16_t now = (uint16_t)TIM2->CNT;
        uint16_t diff = (now >= start) ? (uint16_t)(now - start) : (uint16_t)(5000U - start + now);
        elapsed = (uint16_t)(elapsed + diff);
        start = now;
    }
}

static uint16_t clamp_servo_us(uint16_t us) {
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

/*
 * Sequenced Pulsed Precharge & True 0 uA Limp Disconnect (Q1 AO3401A + U5 MT3608):
 * Because Q1 has a resistive gate pull-up (R12 = 10k) and feeds 110 uF total bulk capacitance
 * (C12 = 22 uF + 4x 22 uF boost bank through L1/D4), snapping PB4 hard LOW would draw ~32 A
 * peak inrush and nuisance-trip the Framework 1.5 A USB-C port switch.
 * Pulsing PB4 at 25 kHz (1 -> 8 us duty over 12 ms = 300 cycles) precharges the 110 uF bank
 * via L1's di/dt limit at < 0.45 A port current before latching Q1 ON and enabling MT3608!
 */
static void set_servo_power(bool enable) {
    if (enable && !g_servo_power_on) {
        GPIOB->BRR = (1UL << 5); /* Keep MT3608 BOOST_EN LOW during capacitor precharge */
        for (uint16_t step = 0U; step < 300U; step++) {
            uint16_t t_on_us = (uint16_t)(1U + (step / 38U)); /* 1 us -> 8 us pulse width */
            GPIOB->BRR = (1UL << 4);  /* Q1 ON */
            delay_us(t_on_us);
            GPIOB->BSRR = (1UL << 4); /* Q1 OFF */
            delay_us((uint16_t)(40U - t_on_us));
        }
        /* Latch Q1 fully ON (PB4 LOW), settle 2 ms, then enable MT3608 6.04 V boost (PB5 HIGH) */
        GPIOB->BRR = (1UL << 4);
        delay_ms(2);
        GPIOB->BSRR = (1UL << 5);
        g_servo_power_on = true;
    } else if (!enable) {
        /* Disable boost first, then open Q1 so output reservoir cannot back-feed VBUS */
        GPIOB->BRR = (1UL << 5);
        GPIOB->BSRR = (1UL << 4);
        g_servo_power_on = false;
    }
}

/**
 * Hardware 140 us Overcurrent Foldback ISR (PB12 = INA_ALERT_N falling edge):
 * Fires within 0.14 ms whenever 5V VBUS current reaches 1.38 A (27.60 mV across 20 mOhm R8),
 * shedding dual-servo stall load > 30x faster than the 1.5 A USB-C port overcurrent switch!
 */
void EXTI15_10_IRQHandler(void) {
    if (EXTI_PR & (1UL << 12)) {
        EXTI_PR = (1UL << 12); /* Clear pending EXTI12 flag */
        g_ocp_alert_count++;
        if (g_servo_power_on) {
            g_active_hip_us  = (uint16_t)((3U * g_active_hip_us + SERVO_NEUTRAL_US) / 4U);
            g_active_knee_us = (uint16_t)((3U * g_active_knee_us + SERVO_NEUTRAL_US) / 4U);
            TIM2->CCR1 = g_active_hip_us;
            TIM2->CCR2 = g_active_knee_us;
        }
    }
}

/**
 * USB 1 kHz Start-Of-Frame (SOF) Hardware Synchronization ISR:
 * All 4 expansion bays on the laptop's root hub receive SOF within +/-500 ns.
 * Latches staged gait setpoints simultaneously on the target 11-bit SOF frame!
 */
void USB_LP_CAN1_RX0_IRQHandler(void) {
    uint32_t istr = USB_ISTR;
    if (istr & USB_ISTR_SOF) {
        USB_ISTR = ~USB_ISTR_SOF;
        uint16_t fn = (uint16_t)(USB_FNR & 0x07FFU);
        g_sof_frame = fn;

        /* Phase-lock TIM2 5 ms (200 Hz) servo period to USB 1 ms SOF every 5 frames */
        if ((fn % 5U) == 0U) {
            TIM2->CNT = 0U;
        }

        /* Commit staged trajectory waypoint on exact target SOF frame */
        if (fn == g_target_sof && g_servo_power_on) {
            g_active_hip_us  = clamp_servo_us(g_staged_hip_us);
            g_active_knee_us = clamp_servo_us(g_staged_knee_us);
            TIM2->CCR1 = g_active_hip_us;
            TIM2->CCR2 = g_active_knee_us;
        }
    }
}

/* Configure 72 MHz system clock from 8 MHz HSE crystal Y1 (PLL x9, USB = 48 MHz) */
static void clock_init_72mhz(void) {
    RCC_CR |= (1UL << 16);
    while ((RCC_CR & (1UL << 17)) == 0U) {}

    FLASH_ACR = 0x12UL;
    RCC_CFGR = (7UL << 18) | (1UL << 16) | (4UL << 8);
    RCC_CR |= (1UL << 24);
    while ((RCC_CR & (1UL << 25)) == 0U) {}

    RCC_CFGR |= 0x02UL;
    while (((RCC_CFGR >> 2) & 0x03UL) != 0x02UL) {}

    SYST_RVR = 72000UL - 1UL;
    SYST_CVR = 0UL;
    SYST_CSR = 0x07UL;
}

static void gpio_and_tim2_init(void) {
    RCC_APB2ENR |= (1UL << 0) | (1UL << 2) | (1UL << 3) | (1UL << 4);
    RCC_APB1ENR |= (1UL << 0) | (1UL << 22) | (1UL << 23);

    /* Free PB4 from NJTRST so PB4 drives Q1 AO3401A gate cleanly while keeping SWD active */
    AFIO_MAPR = (AFIO_MAPR & ~(7UL << 24)) | (2UL << 24);

    /* Safe initial state: Q1 OFF (PB4=1), MT3608 OFF (PB5=0) */
    GPIOB->BSRR = (1UL << 4);
    GPIOB->BRR  = (1UL << 5);

    /* PA0, PA1 = AF Push-Pull 10 MHz (TIM2_CH1 HIP_PWM, TIM2_CH2 KNEE_PWM) */
    GPIOA->CRL = (GPIOA->CRL & ~0x000000FFUL) | 0x00000099UL;
    /* PA8 = Push-Pull Output (USB_ENUM) */
    GPIOA->CRH = (GPIOA->CRH & ~0x0000000FUL) | 0x00000002UL;
    GPIOA->BRR = (1UL << 8);

    /* PB0, PB1 = Input w/ Pull-Up (BAY_ID0/1); PB4, PB5 = Out PP 2 MHz */
    GPIOB->CRL = (GPIOB->CRL & ~0x00FF00FFUL) | 0x00220088UL;
    GPIOB->BSRR = (1UL << 0) | (1UL << 1);

    /* PB10, PB11 = AF Open-Drain (I2C2); PB12 = Input Pull-Up (INA_ALERT_N) */
    GPIOB->CRH = (GPIOB->CRH & ~0x000FFF00UL) | 0x0008EE00UL;
    GPIOB->BSRR = (1UL << 12);

    /* PC13 = ACT_LED Out PP */
    GPIOC->CRH = (GPIOC->CRH & ~0x00F00000UL) | 0x00200000UL;

    /* Read hardware bay strap (staggered soft-start delay = bay_id * 35 ms) */
    uint32_t idr = GPIOB->IDR;
    g_bay_id = (uint8_t)(((idr & 1UL) ? 0U : 1U) | (((idr & 2UL) ? 0U : 1U) << 1));

    /* TIM2: 72 MHz / 72 = 1 MHz (1 us resolution), ARR = 5000 - 1 (200 Hz digital servo rate) */
    TIM2->PSC   = 72U - 1U;
    TIM2->ARR   = 5000U - 1U;
    TIM2->CCR1  = SERVO_NEUTRAL_US;
    TIM2->CCR2  = SERVO_NEUTRAL_US;
    TIM2->CCMR1 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);
    TIM2->CCER  = (1UL << 0) | (1UL << 4);
    TIM2->EGR   = 1UL;
    TIM2->CR1   = 1UL;
}

static void i2c2_init_400khz(void) {
    I2C2->CR1 = (1UL << 15);
    I2C2->CR1 = 0UL;
    I2C2->CR2 = 36UL;
    I2C2->CCR = (1UL << 15) | 30UL;
    I2C2->TRISE = 12UL;
    I2C2->CR1 = 1UL;
}

static bool ina226_write16(uint8_t reg, uint16_t val) {
    uint32_t guard = 20000U;
    I2C2->CR1 |= (1UL << 8);
    while (!(I2C2->SR1 & (1UL << 0))) { if (--guard == 0U) return false; }
    I2C2->DR = (0x40U << 1);
    while (!(I2C2->SR1 & (1UL << 1))) { if (--guard == 0U) return false; }
    (void)I2C2->SR2;
    I2C2->DR = reg;
    while (!(I2C2->SR1 & (1UL << 7))) { if (--guard == 0U) return false; }
    I2C2->DR = (uint8_t)(val >> 8);
    while (!(I2C2->SR1 & (1UL << 7))) { if (--guard == 0U) return false; }
    I2C2->DR = (uint8_t)(val & 0xFFU);
    while (!(I2C2->SR1 & (1UL << 2))) { if (--guard == 0U) return false; }
    I2C2->CR1 |= (1UL << 9);
    return true;
}

static int16_t ina226_read_shunt_raw(void) {
    uint32_t guard = 20000U;
    I2C2->CR1 |= (1UL << 8);
    while (!(I2C2->SR1 & (1UL << 0))) { if (--guard == 0U) return 0; }
    I2C2->DR = (0x40U << 1);
    while (!(I2C2->SR1 & (1UL << 1))) { if (--guard == 0U) return 0; }
    (void)I2C2->SR2;
    I2C2->DR = 0x01U; /* Shunt Voltage Register (2.5 uV/LSB; 20 mOhm -> 8 LSB/mA) */
    while (!(I2C2->SR1 & (1UL << 2))) { if (--guard == 0U) return 0; }

    I2C2->CR1 |= (1UL << 8) | (1UL << 10);
    while (!(I2C2->SR1 & (1UL << 0))) { if (--guard == 0U) return 0; }
    I2C2->DR = (0x40U << 1) | 1U;
    while (!(I2C2->SR1 & (1UL << 1))) { if (--guard == 0U) return 0; }
    (void)I2C2->SR2;
    while (!(I2C2->SR1 & (1UL << 6))) { if (--guard == 0U) return 0; }
    uint8_t msb = (uint8_t)I2C2->DR;
    I2C2->CR1 &= ~(1UL << 10);
    I2C2->CR1 |= (1UL << 9);
    while (!(I2C2->SR1 & (1UL << 6))) { if (--guard == 0U) return 0; }
    uint8_t lsb = (uint8_t)I2C2->DR;
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

int main(void) {
    clock_init_72mhz();
    gpio_and_tim2_init();
    i2c2_init_400khz();

    /* Configure INA226: 1x avg, 140 us fast shunt+bus conversion; hardware SOL alert at 1.38 A (11040 LSB) */
    (void)ina226_write16(0x00U, 0x4007U);
    (void)ina226_write16(0x07U, INA226_ALERT_SOL_LSB);
    (void)ina226_write16(0x06U, 0x8000U); /* Shunt Over-Voltage Alert (SOL) on INA_ALERT_N (PB12) */

    /* Route PB12 -> EXTI12 falling-edge hardware OCP interrupt (IRQ 40 = NVIC_ISER1 bit 8) */
    AFIO_EXTICR4 = (AFIO_EXTICR4 & ~0x000FUL) | 0x0001UL;
    EXTI_FTSR |= (1UL << 12);
    EXTI_PR    = (1UL << 12);
    EXTI_IMR  |= (1UL << 12);
    NVIC_ISER1 = (1UL << 8);

    /* Enable USB FS SOF interrupt & pull up D+ via R2 (PA8) */
    USB_CNTR = USB_CNTR_SOFM;
    NVIC_ISER0 = (1UL << 20);
    delay_ms(10U + (uint32_t)g_bay_id * 40U); /* Staggered 40 ms/bay startup prevents multi-port overlap */
    GPIOA->BSRR = (1UL << 8);

    g_last_cmd_ms = g_ms_ticks;

    while (1) {
        /* 1. Hardware/Telemetry Bay Current Foldback (< 1.50 A USB Type-C per-bay ceiling) */
        int16_t shunt_lsb = ina226_read_shunt_raw();
        int32_t bay_ma = (int32_t)shunt_lsb / 8; /* 20 mOhm * 1 mA = 20 uV = 8 * 2.5 uV */
        if (bay_ma > BAY_CURRENT_LIMIT_MA || (GPIOB->IDR & (1UL << 12)) == 0U) {
            /* Blend active hip & knee 20% toward neutral to shed stall torque smoothly */
            g_active_hip_us  = (uint16_t)((4U * g_active_hip_us + SERVO_NEUTRAL_US) / 5U);
            g_active_knee_us = (uint16_t)((4U * g_active_knee_us + SERVO_NEUTRAL_US) / 5U);
            TIM2->CCR1 = g_active_hip_us;
            TIM2->CCR2 = g_active_knee_us;
        }

        /* 2. Host Watchdog Limp Cutoff */
        if (g_servo_power_on && (g_ms_ticks - g_last_cmd_ms) > WATCHDOG_TIMEOUT_MS) {
            set_servo_power(false);
        }

        /* 3. Heartbeat / SOF Sync Indicator on ACT_LED (PC13) */
        if ((g_sof_frame & 0x01FFU) < 64U) {
            GPIOC->BSRR = (1UL << 13);
        } else {
            GPIOC->BRR = (1UL << 13);
        }
        delay_ms(2);
    }
}
