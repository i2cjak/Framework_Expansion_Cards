/**
 * Day 05 — USB-C I2C/Qwiic Bridge & 36V/5A Power Telemetry Probe
 * Target MCU: STM32F103C8T6 (8 MHz HSE -> 72 MHz SYSCLK, 48 MHz USB FS)
 *
 * Hardware Pin Map (matches Day_05_Power_Bus_Probe.kicad_sch):
 *   PC13 : ACT_LED          (Active-High Red Status LED via 1k R14)
 *   PA8  : USB_ENUM         (Active-High 1.5k R2 D+ Full-Speed Pull-Up)
 *   PA11 : USB_DM           (USB Full-Speed D- via 22R R4)
 *   PA12 : USB_DP           (USB Full-Speed D+ via 22R R3)
 *   PA13 : SWDIO            (J4 SWD Debug Header)
 *   PA14 : SWCLK            (J4 SWD Debug Header)
 *   PB4  : QWIIC_PWR_EN_N   (Active-Low AO3401A Q1 High-Side 3.3V Qwiic Switch)
 *   PB5  : QWIIC_PU_EN      (High = Enable 4.7k R5/R6 Pull-Ups; Hi-Z = Disable)
 *   PB6  : QWIIC_SCL        (I2C1_SCL -> J2 Qwiic 4-pin 1.0mm Connector)
 *   PB7  : QWIIC_SDA        (I2C1_SDA -> J2 Qwiic 4-pin 1.0mm Connector)
 *   PB10 : INA_SCL          (I2C2_SCL -> U4 INA226AIDGSR @ 0x40)
 *   PB11 : INA_SDA          (I2C2_SDA -> U4 INA226AIDGSR @ 0x40)
 *   PB12 : INA_ALERT_N      (Active-Low Alert Input from U4 INA226 ~Alert)
 */

#include <stdbool.h>
#include <stdint.h>

/* --- Minimal STM32F103 Register Definitions --- */
#define PERIPH_BASE       0x40000000UL
#define APB1PERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE   (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE    (PERIPH_BASE + 0x00020000UL)

#define RCC_BASE          (AHBPERIPH_BASE + 0x00001000UL)
#define FLASH_R_BASE      (AHBPERIPH_BASE + 0x00002000UL)
#define AFIO_BASE         (APB2PERIPH_BASE + 0x00000000UL)
#define GPIOA_BASE        (APB2PERIPH_BASE + 0x00000800UL)
#define GPIOB_BASE        (APB2PERIPH_BASE + 0x00000C00UL)
#define GPIOC_BASE        (APB2PERIPH_BASE + 0x00001000UL)
#define I2C1_BASE         (APB1PERIPH_BASE + 0x00005400UL)
#define I2C2_BASE         (APB1PERIPH_BASE + 0x00005800UL)
#define USB_BASE          (APB1PERIPH_BASE + 0x00005C00UL)

#define REG32(addr)       (*(volatile uint32_t *)(addr))
#define REG16(addr)       (*(volatile uint16_t *)(addr))

/* RCC */
#define RCC_CR            REG32(RCC_BASE + 0x00)
#define RCC_CFGR          REG32(RCC_BASE + 0x04)
#define RCC_APB2ENR       REG32(RCC_BASE + 0x18)
#define RCC_APB1ENR       REG32(RCC_BASE + 0x1C)

/* FLASH */
#define FLASH_ACR         REG32(FLASH_R_BASE + 0x00)

/* AFIO */
#define AFIO_MAPR         REG32(AFIO_BASE + 0x04)

/* SysTick */
#define SYST_CSR          REG32(0xE000E010UL)
#define SYST_RVR          REG32(0xE000E014UL)
#define SYST_CVR          REG32(0xE000E018UL)

typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} GPIO_TypeDef;

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

#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define I2C1  ((I2C_TypeDef *)I2C1_BASE)
#define I2C2  ((I2C_TypeDef *)I2C2_BASE)

#define USB_CNTR          REG16(USB_BASE + 0x40)
#define USB_ISTR          REG16(USB_BASE + 0x44)
#define USB_DADDR         REG16(USB_BASE + 0x4C)

/* INA226 I2C Registers (7-bit address 0x40: A1=GND, A0=GND) */
#define INA226_ADDR       0x40U
#define INA226_REG_CONFIG 0x00U
#define INA226_REG_SHUNT  0x01U
#define INA226_REG_BUSV   0x02U
#define INA226_REG_POWER  0x03U
#define INA226_REG_CURR   0x04U
#define INA226_REG_CALIB  0x05U

/*
 * Shunt: R8 = 20 mOhm (0.020 Ohm), Max expected current = 5.0 A.
 * Choose Current_LSB = 200 uA/bit (0.0002 A):
 *   CAL = 0.00512 / (Current_LSB * R_shunt)
 *       = 0.00512 / (0.0002 * 0.020) = 1280 (0x0500)
 *   Power_LSB = 25 * Current_LSB = 5.0 mW/bit
 *   BusV_LSB  = 1.25 mV/bit
 */
#define INA226_CAL_20MR_200UA  0x0500U

typedef struct {
    int32_t  shunt_uv;
    uint32_t bus_mv;
    int32_t  current_ua;
    uint32_t power_mw;
    bool     alert_active;
    uint8_t  qwiic_devices_found;
    uint8_t  qwiic_addr_bitmap[16];
} ProbeTelemetry_t;

static volatile uint32_t g_ms_ticks = 0;
static ProbeTelemetry_t  g_telemetry;

void SysTick_Handler(void) {
    g_ms_ticks++;
}

static void delay_ms(uint32_t ms) {
    uint32_t start = g_ms_ticks;
    while ((g_ms_ticks - start) < ms) {
        __asm volatile("nop");
    }
}

static void clock_init_72mhz(void) {
    /* Enable 8 MHz HSE crystal Y1 */
    RCC_CR |= (1UL << 16);
    while ((RCC_CR & (1UL << 17)) == 0) {}

    /* 2 Flash wait states for 72 MHz + Prefetch buffer */
    FLASH_ACR = 0x12UL;

    /* HCLK = SYSCLK, PCLK2 = HCLK (72 MHz), PCLK1 = HCLK/2 (36 MHz),
     * USBPRE = 0 (PLL / 1.5 = 48 MHz), PLLSRC = HSE, PLLMUL = x9 (72 MHz) */
    RCC_CFGR = (0x4UL << 8) | (1UL << 16) | (0x7UL << 18);

    /* Enable PLL */
    RCC_CR |= (1UL << 24);
    while ((RCC_CR & (1UL << 25)) == 0) {}

    /* Switch SYSCLK to PLL */
    RCC_CFGR = (RCC_CFGR & ~0x3UL) | 0x2UL;
    while (((RCC_CFGR >> 2) & 0x3UL) != 0x2UL) {}

    /* 1 kHz SysTick (72,000,000 / 1000 - 1) */
    SYST_RVR = 72000UL - 1UL;
    SYST_CVR = 0;
    SYST_CSR = 0x7UL;
}

static void gpio_init(void) {
    /* Enable AFIO, GPIOA, GPIOB, GPIOC clocks */
    RCC_APB2ENR |= (1UL << 0) | (1UL << 2) | (1UL << 3) | (1UL << 4);

    /* Disable JTAG while keeping SWD active (frees PB4 for QWIIC_PWR_EN_N!) */
    AFIO_MAPR = (AFIO_MAPR & ~(0x7UL << 24)) | (0x2UL << 24);

    /* PC13: Push-Pull Output 2 MHz (ACT_LED) */
    GPIOC->CRH = (GPIOC->CRH & ~(0xFUL << 20)) | (0x2UL << 20);
    GPIOC->BRR = (1UL << 13);

    /* PA8: Push-Pull Output 2 MHz (USB_ENUM D+ 1.5k pull-up, initially low) */
    GPIOA->CRH = (GPIOA->CRH & ~(0xFUL << 0)) | (0x2UL << 0);
    GPIOA->BRR = (1UL << 8);

    /* PB4 (QWIIC_PWR_EN_N): Output Open-Drain/Push-Pull, initially HIGH (OFF) */
    GPIOB->BSRR = (1UL << 4);
    /* PB5 (QWIIC_PU_EN): Push-Pull Output (HIGH enables 4.7k pull-ups) */
    GPIOB->BSRR = (1UL << 5);
    /* PB6/PB7 (I2C1 SCL/SDA): Alternate Function Open-Drain 10 MHz (0xD) */
    GPIOB->CRL = (GPIOB->CRL & ~0xFFFF0000UL) | 0xDD220000UL;

    /* PB10/PB11 (I2C2 SCL/SDA): Alternate Function Open-Drain 10 MHz (0xD)
     * PB12 (INA_ALERT_N): Input with Pull-Up (0x8 + ODR=1) */
    GPIOB->CRH = (GPIOB->CRH & ~0x000FFF00UL) | 0x0008DD00UL;
    GPIOB->BSRR = (1UL << 12);
}

static void qwiic_set_power(bool enable_3v3, bool enable_pullups) {
    if (enable_pullups) {
        GPIOB->CRL = (GPIOB->CRL & ~(0xFUL << 20)) | (0x2UL << 20);
        GPIOB->BSRR = (1UL << 5);
    } else {
        /* Float PB5 Hi-Z so R5/R6 do not load external 1.8V/external pull-ups */
        GPIOB->CRL = (GPIOB->CRL & ~(0xFUL << 20)) | (0x4UL << 20);
    }

    if (enable_3v3) {
        GPIOB->BRR = (1UL << 4);  /* Active-low AO3401A gate ON */
    } else {
        GPIOB->BSRR = (1UL << 4); /* OFF */
    }
}

static void i2c_bus_init(I2C_TypeDef *i2c, uint32_t speed_khz) {
    /* APB1 = 36 MHz */
    i2c->CR1 = (1UL << 15); /* SWRST */
    i2c->CR1 = 0;
    i2c->CR2 = 36U;         /* 36 MHz peripheral clock */
    if (speed_khz <= 100U) {
        i2c->CCR = 180U;    /* 36 MHz / (2 * 180) = 100 kHz */
        i2c->TRISE = 37U;
    } else {
        i2c->CCR = (1UL << 15) | 30U; /* Fast-mode 400 kHz */
        i2c->TRISE = 11U;
    }
    i2c->CR1 = (1UL << 0);  /* PE = 1 */
}

static bool i2c_wait_flag(volatile uint32_t *reg, uint32_t mask) {
    for (uint32_t t = 0; t < 20000U; t++) {
        if ((*reg & mask) != 0U) {
            return true;
        }
    }
    return false;
}

static bool i2c_probe_addr(I2C_TypeDef *i2c, uint8_t addr7) {
    i2c->SR1 &= ~(1UL << 10); /* Clear AF */
    i2c->CR1 |= (1UL << 8);   /* START */
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 0))) {
        return false;
    }
    i2c->DR = (uint32_t)(addr7 << 1);
    for (uint32_t t = 0; t < 20000U; t++) {
        uint32_t sr1 = i2c->SR1;
        if (sr1 & (1UL << 1)) { /* ADDR */
            (void)i2c->SR2;
            i2c->CR1 |= (1UL << 9); /* STOP */
            return true;
        }
        if (sr1 & (1UL << 10)) { /* AF (NACK) */
            i2c->SR1 &= ~(1UL << 10);
            i2c->CR1 |= (1UL << 9);
            return false;
        }
    }
    i2c->CR1 |= (1UL << 9);
    return false;
}

static bool i2c_write_reg16(I2C_TypeDef *i2c, uint8_t addr7, uint8_t reg, uint16_t val) {
    i2c->CR1 |= (1UL << 8);
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 0))) return false;
    i2c->DR = (uint32_t)(addr7 << 1);
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 1))) { i2c->CR1 |= (1UL << 9); return false; }
    (void)i2c->SR2;

    i2c->DR = reg;
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 7))) return false;
    i2c->DR = (val >> 8) & 0xFFU;
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 7))) return false;
    i2c->DR = val & 0xFFU;
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 2))) return false; /* BTF */
    i2c->CR1 |= (1UL << 9);
    return true;
}

static bool i2c_read_reg16(I2C_TypeDef *i2c, uint8_t addr7, uint8_t reg, uint16_t *out) {
    i2c->CR1 |= (1UL << 8);
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 0))) return false;
    i2c->DR = (uint32_t)(addr7 << 1);
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 1))) { i2c->CR1 |= (1UL << 9); return false; }
    (void)i2c->SR2;

    i2c->DR = reg;
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 2))) return false;

    /* Repeated START for read */
    i2c->CR1 |= (1UL << 8) | (1UL << 10); /* START + ACK */
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 0))) return false;
    i2c->DR = (uint32_t)((addr7 << 1) | 1U);
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 1))) { i2c->CR1 |= (1UL << 9); return false; }
    (void)i2c->SR2;

    if (!i2c_wait_flag(&i2c->SR1, (1UL << 6))) return false;
    uint8_t msb = (uint8_t)i2c->DR;
    i2c->CR1 &= ~(1UL << 10); /* NACK last byte */
    i2c->CR1 |= (1UL << 9);   /* STOP */
    if (!i2c_wait_flag(&i2c->SR1, (1UL << 6))) return false;
    uint8_t lsb = (uint8_t)i2c->DR;

    *out = ((uint16_t)msb << 8) | lsb;
    return true;
}

static void ina226_init(void) {
    /* 16x averaging, 1.1ms bus & shunt conversion, continuous shunt + bus mode (0x4527) */
    (void)i2c_write_reg16(I2C2, INA226_ADDR, INA226_REG_CONFIG, 0x4527U);
    (void)i2c_write_reg16(I2C2, INA226_ADDR, INA226_REG_CALIB, INA226_CAL_20MR_200UA);
}

static void ina226_poll(ProbeTelemetry_t *t) {
    uint16_t raw_shunt = 0, raw_bus = 0, raw_curr = 0, raw_pwr = 0;
    if (i2c_read_reg16(I2C2, INA226_ADDR, INA226_REG_SHUNT, &raw_shunt)) {
        /* 2.5 uV/bit -> * 5 / 2 */
        t->shunt_uv = ((int32_t)(int16_t)raw_shunt * 5) / 2;
    }
    if (i2c_read_reg16(I2C2, INA226_ADDR, INA226_REG_BUSV, &raw_bus)) {
        /* 1.25 mV/bit -> * 5 / 4 */
        t->bus_mv = ((uint32_t)raw_bus * 5U) / 4U;
    }
    if (i2c_read_reg16(I2C2, INA226_ADDR, INA226_REG_CURR, &raw_curr)) {
        /* 200 uA/bit */
        t->current_ua = (int32_t)(int16_t)raw_curr * 200;
    }
    if (i2c_read_reg16(I2C2, INA226_ADDR, INA226_REG_POWER, &raw_pwr)) {
        /* 5 mW/bit */
        t->power_mw = (uint32_t)raw_pwr * 5U;
    }
    t->alert_active = ((GPIOB->IDR & (1UL << 12)) == 0U);
}

static void qwiic_scan_bus(ProbeTelemetry_t *t) {
    uint8_t found = 0;
    for (uint8_t i = 0; i < 16U; i++) {
        t->qwiic_addr_bitmap[i] = 0;
    }
    for (uint8_t addr = 0x08U; addr <= 0x77U; addr++) {
        if (i2c_probe_addr(I2C1, addr)) {
            t->qwiic_addr_bitmap[addr >> 3] |= (uint8_t)(1U << (addr & 7U));
            found++;
        }
    }
    t->qwiic_devices_found = found;
}

static void usb_fs_attach(void) {
    /* Enable USB peripheral clock on APB1 (bit 23) */
    RCC_APB1ENR |= (1UL << 23);
    USB_CNTR = 0x0003U; /* FRES + PDWN */
    delay_ms(2);
    USB_CNTR = 0x0001U; /* Exit power-down */
    delay_ms(2);
    USB_ISTR = 0;
    USB_CNTR = (1UL << 10); /* Enable RESETM */

    /* Assert PA8 (USB_ENUM) high to connect R2 (1.5k) to D+ */
    GPIOA->BSRR = (1UL << 8);
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
    uint16_t istr = USB_ISTR;
    if (istr & (1U << 10)) { /* USB Bus Reset */
        USB_ISTR = (uint16_t)~(1U << 10);
        USB_DADDR = 0x0080U; /* Enable USB Function @ Address 0 */
    }
}

int main(void) {
    clock_init_72mhz();
    gpio_init();

    /* Enable I2C1 (bit 21) and I2C2 (bit 22) clocks */
    RCC_APB1ENR |= (1UL << 21) | (1UL << 22);
    i2c_bus_init(I2C1, 100U); /* 100 kHz Qwiic Bridge */
    i2c_bus_init(I2C2, 400U); /* 400 kHz INA226 Telemetry */

    /* Power up Qwiic 3.3V output with internal 4.7k pull-ups enabled */
    qwiic_set_power(true, true);
    delay_ms(10);

    ina226_init();
    qwiic_scan_bus(&g_telemetry);
    usb_fs_attach();

    uint32_t last_scan_ms = 0;
    while (1) {
        ina226_poll(&g_telemetry);

        /* Periodic Qwiic bus discovery every 1000 ms */
        if ((g_ms_ticks - last_scan_ms) >= 1000U) {
            last_scan_ms = g_ms_ticks;
            GPIOC->BSRR = (1UL << 13); /* Pulse ACT_LED during scan */
            qwiic_scan_bus(&g_telemetry);
            GPIOC->BRR = (1UL << 13);
        }
        delay_ms(10);
    }
}
