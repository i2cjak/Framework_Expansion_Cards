/*
 * Day 08 — 1 Mbps ISO-11898 CAN 2.0B Bus Analyzer (`gs_usb` SocketCAN Skeleton)
 * Target MCU: STM32F072CBT6 (Cortex-M0 @ 48 MHz from 8.000 MHz +/-10 ppm HSE + HSI48 USB)
 *
 * Hardware Pin Map:
 *   PF0 / PF1 : 8.000 MHz +/-10 ppm Quartz Crystal (PLL x6 -> 48.000 MHz PCLK for bxCAN)
 *   PA11/PA12 : USB Full-Speed (Dedicated 1 KB PMA @ 0x40006000 — simultaneous with bxCAN!)
 *   PB8 / PB9 : bxCAN RX / TX (AF4) -> TI SN65HVD230DR 3.3V 1 Mbps Transceiver
 *   PB0       : CAN_SILENT (`SN65HVD230DR` Rs pin: 0 = 1 Mbps Active, 1 = Listen-Only Standby)
 *   PB1       : TERM_EN (`TLP172AM` PhotoMOS Solid-State Relay: 1 = 120 Ohm Termination ON)
 *   PA0       : VCAN_ADC (`ADC_IN0`, 100k/10k 11:1 divider reading 0-36.3V CAN harness bus rail)
 *   PA1 / PA2 : CAN_RX_LED / CAN_TX_LED (Bezel status bar)
 *   PA3       : TERM_LED (Bezel 120R indicator)
 */

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))
#define REG16(addr) (*(volatile uint16_t *)(addr))

/* RCC & FLASH */
#define FLASH_ACR       REG32(0x40022000UL)
#define RCC_CR          REG32(0x40021000UL)
#define RCC_CFGR        REG32(0x40021004UL)
#define RCC_APB2ENR     REG32(0x40021018UL)
#define RCC_APB1ENR     REG32(0x4002101CUL)
#define RCC_AHBENR      REG32(0x40021014UL)
#define RCC_CR2         REG32(0x40021034UL)
#define RCC_CFGR3       REG32(0x40021030UL)

/* CRS (USB SOF trim for HSI48) */
#define CRS_CR          REG32(0x40006C00UL)

/* GPIOA & GPIOB */
#define GPIOA_MODER     REG32(0x48000000UL)
#define GPIOA_BSRR      REG32(0x48000018UL)
#define GPIOB_MODER     REG32(0x48000400UL)
#define GPIOB_BSRR      REG32(0x48000418UL)
#define GPIOB_AFRH      REG32(0x48000424UL)

/* ADC1 (V_CAN 0-36.3V Harness Voltage Telemetry on PA0) */
#define ADC1_ISR        REG32(0x40012400UL)
#define ADC1_CR         REG32(0x40012408UL)
#define ADC1_CHSELR     REG32(0x40012428UL)
#define ADC1_DR         REG32(0x40012440UL)

/* bxCAN (0x40006400) */
#define CAN_MCR         REG32(0x40006400UL)
#define CAN_MSR         REG32(0x40006404UL)
#define CAN_TSR         REG32(0x40006408UL)
#define CAN_RF0R        REG32(0x4000640CUL)
#define CAN_IER         REG32(0x40006414UL)
#define CAN_BTR         REG32(0x4000641CUL)
#define CAN_TI0R        REG32(0x40006580UL)
#define CAN_TDT0R       REG32(0x40006584UL)
#define CAN_TDL0R       REG32(0x40006588UL)
#define CAN_TDH0R       REG32(0x4000658CUL)
#define CAN_RI0R        REG32(0x400065B0UL)
#define CAN_RDT0R       REG32(0x400065B4UL)
#define CAN_RDL0R       REG32(0x400065B8UL)
#define CAN_RDH0R       REG32(0x400065BCUL)
#define CAN_FMR         REG32(0x40006600UL)
#define CAN_FS1R        REG32(0x4000660CUL)
#define CAN_FA1R        REG32(0x4000661CUL)
#define CAN_F0R1        REG32(0x40006640UL)
#define CAN_F0R2        REG32(0x40006644UL)

/* USB FS Device (0x40005C00) */
#define USB_CNTR        REG16(0x40005C40UL)
#define USB_ISTR        REG16(0x40005C44UL)
#define USB_BCDR        REG16(0x40005C58UL)

/* NVIC */
#define NVIC_ISER       REG32(0xE000E100UL)

typedef struct {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  channel;
    uint8_t  flags;
    uint8_t  reserved;
    uint8_t  data[8];
} __attribute__((packed)) gs_host_frame_t;

static volatile gs_host_frame_t rx_ring[16];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;
static volatile uint16_t rx_led_ticks = 0;
static volatile uint16_t tx_led_ticks = 0;
static volatile uint16_t vcan_raw_adc = 0;

static void clock_init_48mhz_hse_and_usb(void) {
    /* 1 Flash wait state + Prefetch for 48 MHz */
    FLASH_ACR = (1UL << 4) | 1UL;

    /* Start external 8.000 MHz +/-10 ppm quartz crystal (HSEON) */
    RCC_CR |= (1UL << 16);
    while ((RCC_CR & (1UL << 17)) == 0UL) {}

    /* PLL = HSE * 6 = 48.000 MHz for rock-solid ISO-11898 bxCAN bit timing */
    RCC_CFGR = (1UL << 16) | (4UL << 18); /* PLLSRC=HSE, PLLMUL=x6 */
    RCC_CR |= (1UL << 24);
    while ((RCC_CR & (1UL << 25)) == 0UL) {}
    RCC_CFGR = (RCC_CFGR & ~3UL) | 2UL;   /* SW = PLL */
    while (((RCC_CFGR >> 2) & 3UL) != 2UL) {}

    /* Start HSI48 + CRS auto-trim for dedicated USB Full-Speed peripheral */
    RCC_CR2 |= (1UL << 16) | 1UL;         /* HSI48ON + HSI14ON (for ADC) */
    while ((RCC_CR2 & (1UL << 17)) == 0UL) {}
    RCC_CFGR3 &= ~(1UL << 7);             /* USBSW = HSI48 */
    RCC_APB1ENR |= (1UL << 27) | (1UL << 25) | (1UL << 23); /* CRS + CAN + USB */
    CRS_CR |= (1UL << 6) | (1UL << 5);    /* AUTOTRIMEN + CEN */
}

static void gpio_and_adc_init(void) {
    RCC_AHBENR |= (1UL << 17) | (1UL << 18); /* IOPAEN + IOPBEN */
    RCC_APB2ENR |= (1UL << 9);               /* ADCEN */

    /* PA0 = Analog (VCAN_ADC); PA1, PA2, PA3 = Output (CAN_RX_LED, CAN_TX_LED, TERM_LED) */
    GPIOA_MODER = (GPIOA_MODER & ~0x000000FFUL)
                | (3UL << 0) | (1UL << 2) | (1UL << 4) | (1UL << 6);

    /* PB0 = Output (CAN_SILENT=0 Active), PB1 = Output (TERM_EN=0 Off); PB8/PB9 = AF4 (bxCAN) */
    GPIOB_MODER = (GPIOB_MODER & ~0x000F000FUL)
                | (1UL << 0) | (1UL << 2) | (2UL << 16) | (2UL << 18);
    GPIOB_AFRH  = (GPIOB_AFRH & ~0x000000FFUL) | (4UL << 0) | (4UL << 4);
    GPIOB_BSRR  = (1UL << 16) | (1UL << 17); /* Default: Active High-Speed, 120R Term OFF */

    /* Calibrate & enable ADC1 on Channel 0 (PA0) */
    ADC1_CR |= (1UL << 31);
    while (ADC1_CR & (1UL << 31)) {}
    ADC1_CHSELR = (1UL << 0);
    ADC1_CR |= 1UL;
    while ((ADC1_ISR & 1UL) == 0UL) {}
}

void can_set_termination_120r(uint8_t enable) {
    if (enable) {
        GPIOB_BSRR = (1UL << 1);   /* Energize Toshiba TLP172AM PhotoMOS LED */
        GPIOA_BSRR = (1UL << 3);   /* Light bezel TERM_LED */
    } else {
        GPIOB_BSRR = (1UL << 17);
        GPIOA_BSRR = (1UL << 19);
    }
}

void can_set_listen_only(uint8_t silent) {
    if (silent) {
        GPIOB_BSRR = (1UL << 0);   /* SN65HVD230DR Rs = HIGH (Listen-Only Standby) */
    } else {
        GPIOB_BSRR = (1UL << 16);  /* SN65HVD230DR Rs = LOW (1 Mbps High-Speed) */
    }
}

static void bxcan_init_1mbps(uint8_t listen_only) {
    /* Enter initialization mode + automatic bus-off recovery (ABOM) */
    CAN_MCR = (1UL << 6) | 1UL;
    while ((CAN_MSR & 1UL) == 0UL) {}

    /* 48 MHz / (BRP=3 * 16 TQ) = 1.000000 Mbps; TS1=13 TQ, TS2=2 TQ -> 87.5% CiA sample point */
    uint32_t btr = ((2UL - 1UL) << 20) | ((13UL - 1UL) << 16) | (3UL - 1UL);
    if (listen_only) {
        btr |= (1UL << 31); /* SILM */
    }
    CAN_BTR = btr;
    can_set_listen_only(listen_only);

    /* Accept-all 32-bit mask filter into FIFO0 */
    CAN_FMR |= 1UL;
    CAN_FS1R |= 1UL;
    CAN_F0R1 = 0x00000000UL;
    CAN_F0R2 = 0x00000000UL;
    CAN_FA1R |= 1UL;
    CAN_FMR &= ~1UL;

    CAN_IER = (1UL << 1); /* FMPIE0 */
    CAN_MCR &= ~1UL;      /* Leave init mode -> Normal bus operation */
}

int can_transmit_frame(const gs_host_frame_t *f) {
    if ((CAN_TSR & (1UL << 26)) == 0UL) {
        return -1; /* Mailbox 0 busy */
    }
    CAN_TDT0R = f->can_dlc & 0x0FUL;
    CAN_TDL0R = ((uint32_t)f->data[3] << 24) | ((uint32_t)f->data[2] << 16)
              | ((uint32_t)f->data[1] << 8)  | (uint32_t)f->data[0];
    CAN_TDH0R = ((uint32_t)f->data[7] << 24) | ((uint32_t)f->data[6] << 16)
              | ((uint32_t)f->data[5] << 8)  | (uint32_t)f->data[4];

    uint32_t tir = 1UL; /* TXRQ */
    if (f->can_id & 0x80000000UL) {
        tir |= ((f->can_id & 0x1FFFFFFFUL) << 3) | (1UL << 2); /* Extended 29-bit IDE */
    } else {
        tir |= ((f->can_id & 0x7FFUL) << 21);                  /* Standard 11-bit ID */
    }
    CAN_TI0R = tir;
    GPIOA_BSRR = (1UL << 2);
    tx_led_ticks = 8000U;
    return 0;
}

void CEC_CAN_IRQHandler(void) {
    while ((CAN_RF0R & 3UL) != 0UL) {
        uint32_t rir = CAN_RI0R;
        uint32_t rdtr = CAN_RDT0R;
        uint32_t rdlr = CAN_RDL0R;
        uint32_t rdhr = CAN_RDH0R;
        CAN_RF0R = (1UL << 5); /* Release FIFO0 */

        uint8_t next = (uint8_t)((rx_head + 1U) & 15U);
        if (next != rx_tail) {
            rx_ring[rx_head].echo_id = 0xFFFFFFFFUL;
            rx_ring[rx_head].can_id  = (rir & (1UL << 2))
                                     ? (((rir >> 3) & 0x1FFFFFFFUL) | 0x80000000UL)
                                     : ((rir >> 21) & 0x7FFUL);
            rx_ring[rx_head].can_dlc = (uint8_t)(rdtr & 0x0FUL);
            rx_ring[rx_head].channel = 0;
            rx_ring[rx_head].flags   = 0;
            rx_ring[rx_head].data[0] = (uint8_t)(rdlr & 0xFFU);
            rx_ring[rx_head].data[1] = (uint8_t)((rdlr >> 8) & 0xFFU);
            rx_ring[rx_head].data[2] = (uint8_t)((rdlr >> 16) & 0xFFU);
            rx_ring[rx_head].data[3] = (uint8_t)((rdlr >> 24) & 0xFFU);
            rx_ring[rx_head].data[4] = (uint8_t)(rdhr & 0xFFU);
            rx_ring[rx_head].data[5] = (uint8_t)((rdhr >> 8) & 0xFFU);
            rx_ring[rx_head].data[6] = (uint8_t)((rdhr >> 16) & 0xFFU);
            rx_ring[rx_head].data[7] = (uint8_t)((rdhr >> 24) & 0xFFU);
            rx_head = next;
        }
        GPIOA_BSRR = (1UL << 1);
        rx_led_ticks = 8000U;
    }
}

void USB_IRQHandler(void) {
    USB_ISTR = 0; /* Clear pending USB FS endpoint / SOF flags */
}

int main(void) {
    clock_init_48mhz_hse_and_usb();
    gpio_and_adc_init();
    bxcan_init_1mbps(0);

    /* Enable internal 1.5k USB D+ pull-up (DPPU) for Full-Speed gs_usb enumeration */
    USB_CNTR = 0x0000U;
    USB_ISTR = 0x0000U;
    USB_BCDR = (1U << 15);
    NVIC_ISER = (1UL << 30) | (1UL << 31);

    for (;;) {
        /* Sample V_CAN 0-36.3V harness voltage continuously */
        ADC1_CR |= (1UL << 2); /* ADSTART */
        while ((ADC1_ISR & (1UL << 2)) == 0UL) {}
        vcan_raw_adc = (uint16_t)ADC1_DR;

        /* Monostable stretchers for bezel RX/TX LEDs */
        if (rx_led_ticks > 0U) {
            rx_led_ticks--;
            if (rx_led_ticks == 0U) {
                GPIOA_BSRR = (1UL << 17);
            }
        }
        if (tx_led_ticks > 0U) {
            tx_led_ticks--;
            if (tx_led_ticks == 0U) {
                GPIOA_BSRR = (1UL << 18);
            }
        }
    }
}
