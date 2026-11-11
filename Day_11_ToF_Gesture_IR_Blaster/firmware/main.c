/*
 * Day 11 — Dual 4 m Laser Time-of-Flight Stereo Gesture Deck & Universal 940 nm IR Blaster
 * Target: STM32F072CBT6 (Cortex-M0 @ 48 MHz Crystal-Less HSI48 + CRS USB SOF Trim)
 *
 * Pin Mapping (matches Day_11_ToF_Gesture_IR_Blaster.kicad_sch):
 *   PA0  : TOF_LED        (White 0603 ToF Gesture Activity LED)
 *   PA1  : IR_ACT_LED     (Red 0603 IR TX/RX Activity LED)
 *   PA8  : IR_RX_OUT      (TIM1_CH1 AF2, 1 us Input Capture from U6 IRM-H638T/TR2 38 kHz AGC Receiver)
 *   PA11 : USB_DM         (USB 2.0 FS Composite HID Gesture Deck + CDC-ACM Pronto/NEC IR CLI)
 *   PA12 : USB_DP
 *   PB0  : TOF_XSHUT_L    (U4 Left VL53L1CX XSHUT — reassigned to 7-bit I2C addr 0x2A at boot)
 *   PB1  : TOF_GPIO1_L    (U4 Left VL53L1CX active-low Data-Ready IRQ)
 *   PB6  : I2C1_SCL       (AF1, 400 kHz Fast-Mode I2C1 bus)
 *   PB7  : I2C1_SDA       (AF1, 400 kHz Fast-Mode I2C1 bus)
 *   PB9  : IR_OUT         (AF0 Hardware IRTIM = TIM17 36–56 kHz Carrier x TIM16 Envelope -> Q1 AO3400A)
 *   PB10 : TOF_XSHUT_R    (U5 Right VL53L1CX XSHUT — reassigned to 7-bit I2C addr 0x2B at boot)
 *   PB11 : TOF_GPIO1_R    (U5 Right VL53L1CX active-low Data-Ready IRQ)
 */

#include <stdint.h>

#define PERIPH_BASE     0x40000000UL
#define AHB2_BASE       0x48000000UL

#define SYSCFG_BASE     (PERIPH_BASE + 0x00010000UL)
#define TIM1_BASE       (PERIPH_BASE + 0x00012C00UL)
#define TIM16_BASE      (PERIPH_BASE + 0x00014400UL)
#define TIM17_BASE      (PERIPH_BASE + 0x00014800UL)
#define I2C1_BASE       (PERIPH_BASE + 0x00005400UL)
#define USB_BASE        (PERIPH_BASE + 0x00005C00UL)
#define CRS_BASE        (PERIPH_BASE + 0x00006C00UL)
#define RCC_BASE        (PERIPH_BASE + 0x00021000UL)
#define GPIOA_BASE      (AHB2_BASE   + 0x00000000UL)
#define GPIOB_BASE      (AHB2_BASE   + 0x00000400UL)

#define REG32(addr)     (*(volatile uint32_t *)(addr))

/* RCC, CRS & SYSCFG */
#define RCC_CFGR        REG32(RCC_BASE + 0x04UL)
#define RCC_AHBENR      REG32(RCC_BASE + 0x14UL)
#define RCC_APB2ENR     REG32(RCC_BASE + 0x18UL)
#define RCC_APB1ENR     REG32(RCC_BASE + 0x1CUL)
#define RCC_CR2         REG32(RCC_BASE + 0x34UL)
#define CRS_CR          REG32(CRS_BASE + 0x00UL)
#define SYSCFG_CFGR1    REG32(SYSCFG_BASE + 0x00UL)

/* GPIO */
#define GPIOA_MODER     REG32(GPIOA_BASE + 0x00UL)
#define GPIOA_BSRR      REG32(GPIOA_BASE + 0x18UL)
#define GPIOA_AFRH      REG32(GPIOA_BASE + 0x24UL)

#define GPIOB_MODER     REG32(GPIOB_BASE + 0x00UL)
#define GPIOB_OTYPER    REG32(GPIOB_BASE + 0x04UL)
#define GPIOB_IDR       REG32(GPIOB_BASE + 0x10UL)
#define GPIOB_BSRR      REG32(GPIOB_BASE + 0x18UL)
#define GPIOB_AFRL      REG32(GPIOB_BASE + 0x20UL)
#define GPIOB_AFRH      REG32(GPIOB_BASE + 0x24UL)

/* I2C1 */
#define I2C1_CR1        REG32(I2C1_BASE + 0x00UL)
#define I2C1_CR2        REG32(I2C1_BASE + 0x04UL)
#define I2C1_TIMINGR    REG32(I2C1_BASE + 0x10UL)
#define I2C1_ISR        REG32(I2C1_BASE + 0x18UL)
#define I2C1_ICR        REG32(I2C1_BASE + 0x1CUL)
#define I2C1_TXDR       REG32(I2C1_BASE + 0x28UL)

/* TIM1 (1 us IR Learner Capture on PA8) */
#define TIM1_CR1        REG32(TIM1_BASE + 0x00UL)
#define TIM1_SR         REG32(TIM1_BASE + 0x10UL)
#define TIM1_CCMR1      REG32(TIM1_BASE + 0x18UL)
#define TIM1_CCER       REG32(TIM1_BASE + 0x20UL)
#define TIM1_PSC        REG32(TIM1_BASE + 0x28UL)
#define TIM1_ARR        REG32(TIM1_BASE + 0x2CUL)
#define TIM1_CCR1       REG32(TIM1_BASE + 0x34UL)

/* TIM16 (Envelope) & TIM17 (38 kHz Carrier) -> Hardware IRTIM on PB9 */
#define TIM16_CR1       REG32(TIM16_BASE + 0x00UL)
#define TIM16_CCMR1     REG32(TIM16_BASE + 0x18UL)
#define TIM16_CCER      REG32(TIM16_BASE + 0x20UL)
#define TIM16_PSC       REG32(TIM16_BASE + 0x28UL)
#define TIM16_ARR       REG32(TIM16_BASE + 0x2CUL)
#define TIM16_CCR1      REG32(TIM16_BASE + 0x34UL)
#define TIM16_BDTR      REG32(TIM16_BASE + 0x44UL)

#define TIM17_CR1       REG32(TIM17_BASE + 0x00UL)
#define TIM17_CCMR1     REG32(TIM17_BASE + 0x18UL)
#define TIM17_CCER      REG32(TIM17_BASE + 0x20UL)
#define TIM17_PSC       REG32(TIM17_BASE + 0x28UL)
#define TIM17_ARR       REG32(TIM17_BASE + 0x2CUL)
#define TIM17_CCR1      REG32(TIM17_BASE + 0x34UL)
#define TIM17_BDTR      REG32(TIM17_BASE + 0x44UL)

/* USB */
#define USB_CNTR        REG32(USB_BASE + 0x40UL)

#define VL53L1X_DEFAULT_ADDR    0x29U
#define VL53L1X_LEFT_ADDR       0x2AU
#define VL53L1X_RIGHT_ADDR      0x2BU
#define VL53L1X_REG_I2C_SLAVE   0x0001U

static volatile uint16_t last_ir_edge_us;
static volatile uint16_t ir_pulse_width_us;

static void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < (us * 8UL); i++) {
        __asm volatile ("nop");
    }
}

static void clock_init_hsi48_crs(void) {
    RCC_CR2 |= (1UL << 16);
    while ((RCC_CR2 & (1UL << 17)) == 0UL) {
    }
    RCC_CFGR = (RCC_CFGR & ~0x3UL) | 0x3UL;
    while (((RCC_CFGR >> 2) & 0x3UL) != 0x3UL) {
    }
    /* Enable CRS, USB, I2C1 on APB1; SYSCFG, TIM1, TIM16, TIM17 on APB2 */
    RCC_APB1ENR |= (1UL << 27) | (1UL << 23) | (1UL << 21);
    RCC_APB2ENR |= (1UL << 0) | (1UL << 11) | (1UL << 17) | (1UL << 18);
    CRS_CR |= (1UL << 6) | (1UL << 5); /* AUTOTRIMEN | CEN */
}

static void i2c1_write_reg8(uint8_t addr7, uint16_t reg16, uint8_t val) {
    uint32_t timeout = 20000UL;
    I2C1_ICR = 0x3F38UL;
    I2C1_CR2 = ((uint32_t)addr7 << 1) | (3UL << 16) | (1UL << 25) | (1UL << 13);
    uint8_t payload[3] = { (uint8_t)(reg16 >> 8), (uint8_t)(reg16 & 0xFFU), val };
    for (uint32_t i = 0; i < 3UL; i++) {
        while (((I2C1_ISR & (1UL << 1)) == 0UL) && (--timeout > 0UL)) {
        }
        if (timeout == 0UL) {
            return;
        }
        I2C1_TXDR = payload[i];
    }
}

static void dual_vl53l1cx_init(void) {
    /*
     * Hold BOTH VL53L1CX sensors in hardware reset (PB0=0, PB10=0), configure
     * PB6/PB7 as 400 kHz open-drain I2C1 (AF1), then wake Left first -> assign 0x2A,
     * then wake Right -> assign 0x2B.
     */
    GPIOB_BSRR   = (1UL << (0 + 16)) | (1UL << (10 + 16));
    GPIOB_OTYPER |= (1UL << 6) | (1UL << 7);
    GPIOB_AFRL   = (GPIOB_AFRL & ~((0xFUL << 24) | (0xFUL << 28))) | (1UL << 24) | (1UL << 28);
    GPIOB_MODER  = (GPIOB_MODER & ~((3UL << 0) | (3UL << 2) | (3UL << 12) | (3UL << 14) | (3UL << 20) | (3UL << 22)))
                 | (1UL << 0) | (2UL << 12) | (2UL << 14) | (1UL << 20);

    /* 400 kHz Fast-Mode I2C timing @ 48 MHz SYSCLK */
    I2C1_TIMINGR = 0x00901850UL;
    I2C1_CR1     = 1UL;

    delay_us(2000);
    GPIOB_BSRR = (1UL << 0); /* Release Left VL53L1CX XSHUT */
    delay_us(2000);
    i2c1_write_reg8(VL53L1X_DEFAULT_ADDR, VL53L1X_REG_I2C_SLAVE, VL53L1X_LEFT_ADDR);

    GPIOB_BSRR = (1UL << 10); /* Release Right VL53L1CX XSHUT */
    delay_us(2000);
    i2c1_write_reg8(VL53L1X_DEFAULT_ADDR, VL53L1X_REG_I2C_SLAVE, VL53L1X_RIGHT_ADDR);
}

static void irtim_and_capture_init(void) {
    /* PA0 (TOF_LED), PA1 (IR_ACT_LED) = Output; PA8 = AF2 (TIM1_CH1); PB9 = AF0 (IR_OUT) */
    GPIOA_MODER = (GPIOA_MODER & ~((3UL << 0) | (3UL << 2) | (3UL << 16)))
                | (1UL << 0) | (1UL << 2) | (2UL << 16);
    GPIOA_AFRH  = (GPIOA_AFRH & ~(0xFUL << 0)) | (2UL << 0);

    GPIOB_MODER = (GPIOB_MODER & ~(3UL << 18)) | (2UL << 18);
    GPIOB_AFRH  = (GPIOB_AFRH & ~(0xFUL << 4)); /* AF0 = IR_OUT */

    /* Route TIM16 x TIM17 internal AND-gate to PB9 (SYSCFG_CFGR1.IR_MOD = 00) */
    SYSCFG_CFGR1 &= ~(3UL << 6);

    /* TIM17: 38.005 kHz 33% duty carrier @ 48 MHz (ARR = 1262, CCR1 = 421) */
    TIM17_PSC   = 0UL;
    TIM17_ARR   = 1262UL;
    TIM17_CCR1  = 421UL;
    TIM17_CCMR1 = (6UL << 4) | (1UL << 3); /* PWM Mode 1 */
    TIM17_CCER  = 1UL;
    TIM17_BDTR  = (1UL << 15); /* MOE */
    TIM17_CR1   = 1UL;

    /* TIM16: 1 us envelope gate (CCR1 = 0 -> idle OFF so Q1 gate stays safely LOW) */
    TIM16_PSC   = 47UL;
    TIM16_ARR   = 0xFFFFUL;
    TIM16_CCR1  = 0UL;
    TIM16_CCMR1 = (6UL << 4);
    TIM16_CCER  = 1UL;
    TIM16_BDTR  = (1UL << 15); /* MOE */
    TIM16_CR1   = 1UL;

    /* TIM1_CH1 on PA8: 1 us resolution both-edge Input Capture for IRM-H638T/TR2 */
    TIM1_PSC    = 47UL;
    TIM1_ARR    = 0xFFFFUL;
    TIM1_CCMR1  = 1UL;                      /* CC1S = TI1 */
    TIM1_CCER   = 1UL | (1UL << 1) | (1UL << 3); /* CC1E + both edges */
    TIM1_CR1    = 1UL;
}

/* Hardware-gated 38 kHz 940 nm burst helper for NEC / RC5 / Sony SIRC / Pronto frames */
void ir_blast_mark_space_us(uint16_t mark_us, uint16_t space_us) {
    GPIOA_BSRR = (1UL << 1);       /* IR_ACT_LED ON */
    TIM16_CCR1 = 0xFFFFUL;         /* Gate ON 38 kHz TIM17 carrier onto PB9 -> Q1 */
    delay_us(mark_us);
    TIM16_CCR1 = 0UL;              /* Gate OFF carrier */
    GPIOA_BSRR = (1UL << (1 + 16));
    delay_us(space_us);
}

int main(void) {
    RCC_AHBENR |= (1UL << 17) | (1UL << 18); /* GPIOAEN | GPIOBEN */
    clock_init_hsi48_crs();
    dual_vl53l1cx_init();
    irtim_and_capture_init();

    USB_CNTR = 0UL;

    while (1) {
        /* Capture incoming 38 kHz demodulated IR edges from U6 on PA8 (TIM1_CH1) */
        if (TIM1_SR & (1UL << 1)) {
            uint16_t now_us = (uint16_t)TIM1_CCR1;
            ir_pulse_width_us = (uint16_t)(now_us - last_ir_edge_us);
            last_ir_edge_us = now_us;
            GPIOA_BSRR = (1UL << 1);
        } else {
            GPIOA_BSRR = (1UL << (1 + 16));
        }

        /* Light TOF_LED whenever either Left (PB1) or Right (PB11) VL53L1CX signals Data Ready */
        if (((GPIOB_IDR & ((1UL << 1) | (1UL << 11))) != ((1UL << 1) | (1UL << 11)))) {
            GPIOA_BSRR = (1UL << 0);
        } else {
            GPIOA_BSRR = (1UL << (0 + 16));
        }
    }
}
