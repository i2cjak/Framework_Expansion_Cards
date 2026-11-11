/*
 * Day 10 — Autoleveling 1.65V–5.5V Cortex-M CMSIS-DAPv2 SWD Debugger & Dual-Supply UART Bridge
 * Target: STM32F072CBT6 (Cortex-M0 @ 48 MHz Crystal-Less HSI48 + CRS USB SOF Trim)
 *
 * Pin Mapping (matches Day_10_SWD_Debugger_UART.kicad_sch):
 *   PA0  : VTREF_ADC      (ADC_IN0, 2:1 divider measuring 0.00V–6.60V target logic rail)
 *   PA1  : DAP_LED        (White 0603 CMSIS-DAPv2 SWD activity LED)
 *   PA2  : MCU_UART_TX    (USART2_TX -> U7 SN74LVC1T45 -> TGT_UART_RX)
 *   PA3  : MCU_UART_RX    (USART2_RX <- U8 SN74LVC1T45 <- TGT_UART_TX / SWO)
 *   PA4  : UART_LED       (Red 0603 CDC-ACM UART activity LED)
 *   PB0  : MCU_NRST_DRV   (U6 DIR control: 0 = Hi-Z released / sense mode, 1 = drive TGT_NRST)
 *   PB1  : MCU_NRST_IO    (U6 A pin: reads target nRESET when PB0=0; drives 0V open-drain when PB0=1)
 *   PB12 : MCU_SWDIO_DIR  (U4 DIR control: 0 = Target->Probe turnaround, 1 = Probe->Target drive)
 *   PB13 : MCU_SWCLK      (SPI2_SCK / fast GPIO -> U5 -> TGT_SWCLK)
 *   PB14 : MCU_SWDIO      (SPI2_MISO / fast GPIO <-> U4 <-> TGT_SWDIO)
 *   PA11 : USB_DM         (USB 2.0 FS WinUSB CMSIS-DAPv2 + CDC-ACM VCP)
 *   PA12 : USB_DP
 */

#include <stdint.h>

#define PERIPH_BASE     0x40000000UL
#define AHB2_BASE       0x48000000UL

#define RCC_BASE        (PERIPH_BASE + 0x00021000UL)
#define CRS_BASE        (PERIPH_BASE + 0x00006C00UL)
#define ADC_BASE        (PERIPH_BASE + 0x00012400UL)
#define USART2_BASE     (PERIPH_BASE + 0x00004400UL)
#define USB_BASE        (PERIPH_BASE + 0x00005C00UL)
#define GPIOA_BASE      (AHB2_BASE   + 0x00000000UL)
#define GPIOB_BASE      (AHB2_BASE   + 0x00000400UL)

#define REG32(addr)     (*(volatile uint32_t *)(addr))

/* RCC & CRS */
#define RCC_CR2         REG32(RCC_BASE + 0x34UL)
#define RCC_CFGR        REG32(RCC_BASE + 0x04UL)
#define RCC_AHBENR      REG32(RCC_BASE + 0x14UL)
#define RCC_APB2ENR     REG32(RCC_BASE + 0x18UL)
#define RCC_APB1ENR     REG32(RCC_BASE + 0x1CUL)
#define CRS_CR          REG32(CRS_BASE + 0x00UL)

/* GPIO */
#define GPIOA_MODER     REG32(GPIOA_BASE + 0x00UL)
#define GPIOA_BSRR      REG32(GPIOA_BASE + 0x18UL)
#define GPIOA_AFRL      REG32(GPIOA_BASE + 0x20UL)

#define GPIOB_MODER     REG32(GPIOB_BASE + 0x00UL)
#define GPIOB_OSPEEDR   REG32(GPIOB_BASE + 0x08UL)
#define GPIOB_IDR       REG32(GPIOB_BASE + 0x10UL)
#define GPIOB_BSRR      REG32(GPIOB_BASE + 0x18UL)

/* USART2 */
#define USART2_CR1      REG32(USART2_BASE + 0x00UL)
#define USART2_BRR      REG32(USART2_BASE + 0x0CUL)
#define USART2_ISR      REG32(USART2_BASE + 0x1CUL)
#define USART2_RDR      REG32(USART2_BASE + 0x24UL)

/* ADC */
#define ADC_ISR         REG32(ADC_BASE + 0x00UL)
#define ADC_CR          REG32(ADC_BASE + 0x08UL)
#define ADC_CHSELR      REG32(ADC_BASE + 0x28UL)
#define ADC_DR          REG32(ADC_BASE + 0x40UL)

/* USB */
#define USB_CNTR        REG32(USB_BASE + 0x40UL)

static volatile uint16_t vtref_adc_raw;
static volatile uint8_t  last_uart_rx;

static void clock_init_hsi48_crs(void) {
    RCC_CR2 |= (1UL << 16);
    while ((RCC_CR2 & (1UL << 17)) == 0UL) {
    }
    RCC_CFGR = (RCC_CFGR & ~0x3UL) | 0x3UL;
    while (((RCC_CFGR >> 2) & 0x3UL) != 0x3UL) {
    }
    RCC_APB1ENR |= (1UL << 27) | (1UL << 23) | (1UL << 17); /* CRS, USB, USART2 */
    RCC_APB2ENR |= (1UL << 9);                              /* ADC */
    CRS_CR |= (1UL << 6) | (1UL << 5);                      /* AUTOTRIMEN | CEN */
}

static void swd_bridge_gpio_init(void) {
    /*
     * Safe boot state:
     *   PB0  (MCU_NRST_DRV)  = Output LOW (U6 DIR=0 -> TGT_NRST Hi-Z released!)
     *   PB1  (MCU_NRST_IO)   = Input (reads target nRESET level via U6 B->A)
     *   PB12 (MCU_SWDIO_DIR) = Output LOW (U4 DIR=0 -> TGT_SWDIO input mode)
     *   PB13 (MCU_SWCLK)     = Output LOW (high-speed slew)
     *   PB14 (MCU_SWDIO)     = Input (switched to Output when PB12=1)
     */
    GPIOB_BSRR    = (1UL << (0 + 16)) | (1UL << (12 + 16)) | (1UL << (13 + 16));
    GPIOB_OSPEEDR |= (3UL << 24) | (3UL << 26) | (3UL << 28);
    GPIOB_MODER   = (GPIOB_MODER & ~((3UL << 0) | (3UL << 2) | (3UL << 24) | (3UL << 26) | (3UL << 28)))
                  | (1UL << 0) | (1UL << 24) | (1UL << 26);
}

/* Single cycle-clean CMSIS-DAP SWD write/read bit primitive */
uint32_t dap_swd_transfer_bit(uint32_t drive_out, uint32_t out_bit) {
    if (drive_out) {
        /* Switch PB14 to Output + U4 DIR (PB12) HIGH (A -> B) */
        if (out_bit) {
            GPIOB_BSRR = (1UL << 14) | (1UL << 12);
        } else {
            GPIOB_BSRR = (1UL << (14 + 16)) | (1UL << 12);
        }
        GPIOB_MODER = (GPIOB_MODER & ~(3UL << 28)) | (1UL << 28);
    } else {
        /* Switch PB14 to Input BEFORE flipping U4 DIR (PB12) LOW (B -> A) */
        GPIOB_MODER &= ~(3UL << 28);
        GPIOB_BSRR   = (1UL << (12 + 16));
    }

    /* Clock pulse on PB13 (MCU_SWCLK) */
    GPIOB_BSRR = (1UL << (13 + 16));
    __asm volatile ("nop\n\tnop");
    uint32_t sampled = (GPIOB_IDR >> 14) & 1UL;
    GPIOB_BSRR = (1UL << 13);
    return sampled;
}

static void uart2_and_vtref_adc_init(void) {
    /* PA0 = Analog (VTREF_ADC); PA1, PA4 = LED outputs; PA2/PA3 = AF1 (USART2 TX/RX) */
    GPIOA_MODER = (GPIOA_MODER & ~0x000003FFUL)
                | (3UL << 0) | (1UL << 2) | (2UL << 4) | (2UL << 6) | (1UL << 8);
    GPIOA_AFRL  = (GPIOA_AFRL & ~0x0000FF00UL) | (1UL << 8) | (1UL << 12);

    /* 115200 8N1 default @ 48 MHz (48,000,000 / 115,200 = 417) */
    USART2_BRR = 417UL;
    USART2_CR1 = (1UL << 3) | (1UL << 2) | 1UL; /* TE | RE | UE */

    /* HSI14 ADC on PA0 (CH0) */
    RCC_CR2 |= 1UL;
    while ((RCC_CR2 & 2UL) == 0UL) {
    }
    ADC_CR |= 1UL;
    while ((ADC_ISR & 1UL) == 0UL) {
    }
    ADC_CHSELR = (1UL << 0);
}

int main(void) {
    RCC_AHBENR |= (1UL << 17) | (1UL << 18); /* GPIOAEN | GPIOBEN */
    clock_init_hsi48_crs();
    swd_bridge_gpio_init();
    uart2_and_vtref_adc_init();

    USB_CNTR = 0UL;

    uint32_t poll_div = 0UL;
    while (1) {
        if (USART2_ISR & (1UL << 5)) { /* RXNE */
            last_uart_rx = (uint8_t)USART2_RDR;
            GPIOA_BSRR = (1UL << 4);   /* Flash UART_LED */
        }

        poll_div++;
        if ((poll_div & 0xFFFFUL) == 0UL) {
            ADC_CR |= (1UL << 2);
            while ((ADC_ISR & (1UL << 2)) == 0UL) {
            }
            vtref_adc_raw = (uint16_t)ADC_DR;
            /* Illuminate DAP_LED when valid target VTref (>= 1.60V -> ~993 counts) is detected */
            if (vtref_adc_raw >= 993U) {
                GPIOA_BSRR = (1UL << 1);
            } else {
                GPIOA_BSRR = (1UL << (1 + 16));
            }
            GPIOA_BSRR = (1UL << (4 + 16));
        }
    }
}
