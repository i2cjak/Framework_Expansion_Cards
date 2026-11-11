/*
 * Day 09 — Solid-State PIN Radiation Dosimeter & Quantum Decay Hardware RNG
 * Target: STM32F072CBT6 (Cortex-M0 @ 48 MHz Crystal-Less HSI48 + CRS USB SOF Trim)
 *
 * Pin Mapping (matches Day_09_Radiation_Dosimeter_RNG.kicad_sch):
 *   PA0  : HV_MON_ADC   (ADC_IN0, 11:1 Dickson +13.3V depletion bias monitor)
 *   PA1  : PULSE_OUT    (COMP1_INP + ADC_IN1 1 Msps pulse-height spectroscopy)
 *   PA2  : HIT_LED      (Ionization event bezel flash LED)
 *   PA3  : RNG_LED      (Quantum entropy pool ready LED)
 *   PA4  : THRESH_DAC   (DAC_OUT1 / COMP1_INM programmable mV threshold)
 *   PA8  : PUMP_CLK_A   (TIM1_CH1 100 kHz push-pull Dickson pump phase A)
 *   PA9  : PUMP_CLK_B   (TIM1_CH2 100 kHz inverted push-pull Dickson pump phase B)
 *   PA11 : USB_DM       (USB 2.0 FS /dev/hwrng + spectroscopy CDC)
 *   PA12 : USB_DP
 */

#include <stdint.h>

#define PERIPH_BASE     0x40000000UL
#define AHB2_BASE       0x48000000UL

#define RCC_BASE        (PERIPH_BASE + 0x00021000UL)
#define CRS_BASE        (PERIPH_BASE + 0x00006C00UL)
#define SYSCFG_BASE     (PERIPH_BASE + 0x00010000UL)
#define ADC_BASE        (PERIPH_BASE + 0x00012400UL)
#define TIM1_BASE       (PERIPH_BASE + 0x00012C00UL)
#define TIM2_BASE       (PERIPH_BASE + 0x00000000UL)
#define DAC_BASE        (PERIPH_BASE + 0x00007400UL)
#define USB_BASE        (PERIPH_BASE + 0x00005C00UL)
#define GPIOA_BASE      (AHB2_BASE   + 0x00000000UL)

#define REG32(addr)     (*(volatile uint32_t *)(addr))

/* RCC registers */
#define RCC_CR2         REG32(RCC_BASE + 0x34UL)
#define RCC_CFGR        REG32(RCC_BASE + 0x04UL)
#define RCC_AHBENR      REG32(RCC_BASE + 0x14UL)
#define RCC_APB2ENR     REG32(RCC_BASE + 0x18UL)
#define RCC_APB1ENR     REG32(RCC_BASE + 0x1CUL)

/* CRS registers */
#define CRS_CR          REG32(CRS_BASE + 0x00UL)

/* GPIOA registers */
#define GPIOA_MODER     REG32(GPIOA_BASE + 0x00UL)
#define GPIOA_BSRR      REG32(GPIOA_BASE + 0x18UL)
#define GPIOA_AFRH      REG32(GPIOA_BASE + 0x24UL)

/* DAC1 registers */
#define DAC_CR          REG32(DAC_BASE + 0x00UL)
#define DAC_DHR12R1     REG32(DAC_BASE + 0x08UL)

/* COMP1 control/status inside SYSCFG+0x1C */
#define COMP_CSR        REG32(SYSCFG_BASE + 0x1CUL)

/* TIM1 (100 kHz complementary-phase charge pump) */
#define TIM1_CR1        REG32(TIM1_BASE + 0x00UL)
#define TIM1_CCMR1      REG32(TIM1_BASE + 0x18UL)
#define TIM1_CCER       REG32(TIM1_BASE + 0x20UL)
#define TIM1_PSC        REG32(TIM1_BASE + 0x28UL)
#define TIM1_ARR        REG32(TIM1_BASE + 0x2CUL)
#define TIM1_CCR1       REG32(TIM1_BASE + 0x34UL)
#define TIM1_CCR2       REG32(TIM1_BASE + 0x38UL)
#define TIM1_BDTR       REG32(TIM1_BASE + 0x44UL)

/* TIM2 (48 MHz free-running 32-bit Poisson timestamp counter) */
#define TIM2_CR1        REG32(TIM2_BASE + 0x00UL)
#define TIM2_CNT        REG32(TIM2_BASE + 0x24UL)
#define TIM2_PSC        REG32(TIM2_BASE + 0x28UL)
#define TIM2_ARR        REG32(TIM2_BASE + 0x2CUL)

/* ADC registers */
#define ADC_ISR         REG32(ADC_BASE + 0x00UL)
#define ADC_CR          REG32(ADC_BASE + 0x08UL)
#define ADC_CHSELR      REG32(ADC_BASE + 0x28UL)
#define ADC_DR          REG32(ADC_BASE + 0x40UL)

/* USB peripheral */
#define USB_CNTR        REG32(USB_BASE + 0x40UL)

/* Quantum entropy pool & 64-bin pulse-height MCA spectrum */
static volatile uint32_t entropy_pool[16];
static volatile uint32_t pool_write_idx;
static volatile uint32_t mca_bins[64];
static volatile uint32_t total_counts;

static void clock_init_hsi48_crs(void) {
    /* Enable 48 MHz HSI48 RC oscillator + USB SOF Clock Recovery System */
    RCC_CR2 |= (1UL << 16);
    while ((RCC_CR2 & (1UL << 17)) == 0UL) {
    }
    RCC_CFGR = (RCC_CFGR & ~0x3UL) | 0x3UL;
    while (((RCC_CFGR >> 2) & 0x3UL) != 0x3UL) {
    }
    RCC_APB1ENR |= (1UL << 27) | (1UL << 23) | (1UL << 29) | (1UL << 0); /* CRS, USB, DAC, TIM2 */
    RCC_APB2ENR |= (1UL << 0)  | (1UL << 9)  | (1UL << 11);              /* SYSCFG/COMP, ADC, TIM1 */
    CRS_CR |= (1UL << 6) | (1UL << 5);                                   /* AUTOTRIMEN | CEN */
}

static void dickson_pump_init_100khz(void) {
    /* PA8 = TIM1_CH1 (AF2), PA9 = TIM1_CH2 (AF2) in 180-deg anti-phase @ 100 kHz */
    GPIOA_MODER = (GPIOA_MODER & ~((3UL << 16) | (3UL << 18))) | (2UL << 16) | (2UL << 18);
    GPIOA_AFRH  = (GPIOA_AFRH  & ~0xFFUL) | 0x22UL;

    TIM1_PSC   = 0UL;
    TIM1_ARR   = 479UL;     /* 48 MHz / 480 = 100.0 kHz */
    TIM1_CCR1  = 240UL;     /* 50% duty cycle */
    TIM1_CCR2  = 240UL;
    /* CH1 = PWM mode 1 (0x60), CH2 = PWM mode 2 (0x70 -> exact 180-deg phase opposition!) */
    TIM1_CCMR1 = (6UL << 4) | (1UL << 3) | (7UL << 12) | (1UL << 11);
    TIM1_CCER  = (1UL << 0) | (1UL << 4);
    TIM1_BDTR  = (1UL << 15); /* MOE main output enable */
    TIM1_CR1   = 1UL;
}

static void analog_discriminator_init(void) {
    /* PA0, PA1, PA4 = Analog mode; PA2 (HIT_LED), PA3 (RNG_LED) = Outputs */
    GPIOA_MODER |= (3UL << 0) | (3UL << 2) | (3UL << 8);
    GPIOA_MODER  = (GPIOA_MODER & ~((3UL << 4) | (3UL << 6))) | (1UL << 4) | (1UL << 6);

    /* DAC1 Channel 1 on PA4 = 0.680 V threshold (~844 LSB above 0.580 V CSA baseline) */
    DAC_CR      = 1UL;
    DAC_DHR12R1 = 844UL;

    /* COMP1: INP = PA1, INM = DAC1_CH1 (INMSEL = 4), High-speed mode, Enable */
    COMP_CSR = (4UL << 4) | 1UL;

    /* TIM2: 32-bit free-running 48 MHz (20.8 ns) timestamp timer */
    TIM2_PSC = 0UL;
    TIM2_ARR = 0xFFFFFFFFUL;
    TIM2_CR1 = 1UL;

    /* Enable HSI14 ADC for pulse-height capture on PA1 (CH1) */
    RCC_CR2 |= 1UL;
    while ((RCC_CR2 & 2UL) == 0UL) {
    }
    ADC_CR |= 1UL;
    while ((ADC_ISR & 1UL) == 0UL) {
    }
}

static uint16_t sample_pulse_peak_ch1(void) {
    ADC_CHSELR = (1UL << 1);
    ADC_CR |= (1UL << 2);
    while ((ADC_ISR & (1UL << 2)) == 0UL) {
    }
    return (uint16_t)ADC_DR;
}

int main(void) {
    RCC_AHBENR |= (1UL << 17); /* GPIOAEN */
    clock_init_hsi48_crs();
    dickson_pump_init_100khz();
    analog_discriminator_init();

    /* Release USB FS reset bit */
    USB_CNTR = 0UL;

    uint32_t prev_ts = TIM2_CNT;
    uint32_t dt_pair[2] = {0UL, 0UL};
    uint32_t dt_phase = 0UL;
    uint32_t vn_pair = 0UL;
    uint32_t vn_count = 0UL;
    uint32_t entropy_word = 0UL;
    uint32_t entropy_bits = 0UL;
    uint32_t hit_led_hold = 0UL;
    uint32_t prev_comp = 0UL;

    while (1) {
        uint32_t comp_out = (COMP_CSR >> 14) & 1UL;

        /* Rising edge of shaped PIN ionization pulse crossing DAC1 threshold */
        if (comp_out && !prev_comp) {
            uint32_t now = TIM2_CNT;
            uint16_t peak = sample_pulse_peak_ch1();
            mca_bins[(peak >> 6) & 63UL]++;
            total_counts++;

            /* Flash HIT_LED on PA2 */
            GPIOA_BSRR = (1UL << 2);
            hit_led_hold = 24000UL;

            /* Unbiased Poisson inter-arrival interval comparator + Von Neumann whitener */
            uint32_t dt = now - prev_ts;
            prev_ts = now;
            dt_pair[dt_phase] = dt;
            dt_phase ^= 1UL;

            if (dt_phase == 0UL && dt_pair[0] != dt_pair[1]) {
                uint32_t raw_bit = (dt_pair[0] > dt_pair[1]) ? 1UL : 0UL;
                vn_pair = (vn_pair << 1) | raw_bit;
                vn_count++;
                if (vn_count == 2UL) {
                    vn_count = 0UL;
                    if (vn_pair == 1UL || vn_pair == 2UL) {
                        entropy_word = (entropy_word << 1) | (vn_pair & 1UL);
                        entropy_bits++;
                        if (entropy_bits == 32UL) {
                            entropy_bits = 0UL;
                            entropy_pool[pool_write_idx & 15UL] = entropy_word;
                            pool_write_idx++;
                            GPIOA_BSRR = (1UL << 3); /* Assert RNG_LED (PA3) */
                        }
                    }
                    vn_pair = 0UL;
                }
            }
        }
        prev_comp = comp_out;

        if (hit_led_hold > 0UL) {
            hit_led_hold--;
            if (hit_led_hold == 0UL) {
                GPIOA_BSRR = (1UL << (2 + 16));
            }
        }
    }
}
