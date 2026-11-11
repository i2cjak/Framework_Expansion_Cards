/*
 * Day 13 — +22 dBm 868/915 MHz SX1262 LoRa Mesh Radio & Sub-GHz Spectrum Sweeper
 * Target MCU: STM32F072CBT6 (Cortex-M0 @ 48 MHz Crystal-Less HSI48 + CRS USB SOF)
 *
 * Pin Map:
 *   PA4        : LORA_NSS    (SX1262 SPI Chip Select, active low)
 *   PA5..PA7   : SPI1        (SCK, MISO, MOSI @ 12 MHz AF0)
 *   PB0        : LORA_NRESET (SX1262 hardware reset, active low)
 *   PB1        : LORA_BUSY   (SX1262 state-machine busy input)
 *   PB2        : LORA_DIO1   (SX1262 TxDone / RxDone / CadDone interrupt)
 *   PB10       : RF_SW_RX    (Skyworks AS179-92LF V2 RX path enable; V1 <- SX1262 DIO2)
 *   PB4 / PB5  : LED_RX_SWP (Green) / LED_TX_PA (Red)
 *   PA11/PA12  : USB 2.0 FS  (CDC-ACM KISS/RNode TNC & ISM Waterfall CLI)
 */

#include <stdint.h>
#include <stdbool.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))
#define REG16(addr) (*(volatile uint16_t *)(addr))
#define REG8(addr)  (*(volatile uint8_t *)(addr))

/* RCC, CRS, FLASH */
#define FLASH_ACR       REG32(0x40022000UL)
#define RCC_CFGR        REG32(0x40021004UL)
#define RCC_AHBENR      REG32(0x40021014UL)
#define RCC_APB2ENR     REG32(0x40021018UL)
#define RCC_APB1ENR     REG32(0x4002101CUL)
#define RCC_CR2         REG32(0x40021034UL)
#define CRS_CR          REG32(0x40006C00UL)
#define USB_CNTR        REG16(0x40005C40UL)
#define USB_BCDR        REG16(0x40005C58UL)

/* GPIOA & GPIOB */
#define GPIOA_MODER     REG32(0x48000000UL)
#define GPIOA_OSPEEDR   REG32(0x48000008UL)
#define GPIOA_BSRR      REG32(0x48000018UL)

#define GPIOB_MODER     REG32(0x48000400UL)
#define GPIOB_IDR       REG32(0x48000410UL)
#define GPIOB_BSRR      REG32(0x48000418UL)

/* SPI1 */
#define SPI1_CR1        REG32(0x40013000UL)
#define SPI1_CR2        REG32(0x40013004UL)
#define SPI1_SR         REG32(0x40013008UL)
#define SPI1_DR8        REG8(0x4001300CUL)

/* SX1262 Opcodes */
#define SX_CMD_SET_STANDBY          0x80U
#define SX_CMD_SET_TX               0x83U
#define SX_CMD_SET_RX               0x82U
#define SX_CMD_SET_PACKET_TYPE      0x8AU
#define SX_CMD_SET_RF_FREQUENCY     0x86U
#define SX_CMD_SET_PA_CONFIG        0x95U
#define SX_CMD_SET_TX_PARAMS        0x8EU
#define SX_CMD_SET_MOD_PARAMS       0x8BU
#define SX_CMD_SET_PKT_PARAMS       0x8CU
#define SX_CMD_SET_DIO_IRQ_PARAMS   0x08U
#define SX_CMD_SET_DIO2_RF_SW       0x9DU
#define SX_CMD_SET_REGULATOR_MODE   0x96U
#define SX_CMD_CALIBRATE_IMAGE      0x98U
#define SX_CMD_GET_RSSI_INST        0x15U
#define SX_CMD_CLR_IRQ_STATUS       0x02U

/* Operating state & live 128-bin 902..928 MHz US915 waterfall buffer */
static volatile int8_t g_waterfall_dbm[128];
static volatile bool   g_sweep_mode = false;

static void delay_cycles(volatile uint32_t n) {
    while (n--) {
        __asm__ volatile ("nop");
    }
}

static void clock_usb_init(void) {
    FLASH_ACR = 0x11UL;            /* 1 wait state + prefetch @ 48 MHz */
    RCC_CR2 |= (1UL << 16);        /* Enable HSI48 */
    while ((RCC_CR2 & (1UL << 17)) == 0UL) {}
    RCC_CFGR = (RCC_CFGR & ~0x3UL) | 0x3UL;
    while (((RCC_CFGR >> 2) & 0x3UL) != 0x3UL) {}

    RCC_APB1ENR |= (1UL << 27) | (1UL << 23); /* CRS + USB */
    CRS_CR |= (1UL << 6) | (1UL << 5);        /* AUTOTRIMEN + CEN */
    USB_CNTR = (1U << 1);
    delay_cycles(480U);
    USB_CNTR = 0U;
    USB_BCDR = (1U << 15);         /* DP 1.5k internal pull-up */
}

static void gpio_spi1_init(void) {
    RCC_AHBENR  |= (1UL << 17) | (1UL << 18); /* GPIOA + GPIOB */
    RCC_APB2ENR |= (1UL << 12);               /* SPI1 */

    /* PA4 = Output (NSS high), PA5/6/7 = AF0 (SPI1 SCK/MISO/MOSI) */
    GPIOA_BSRR = (1UL << 4);
    GPIOA_OSPEEDR |= (3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) | (3UL << (7 * 2));
    GPIOA_MODER = (GPIOA_MODER & ~(0xFFUL << (4 * 2)))
                | (1UL << (4 * 2)) | (2UL << (5 * 2)) | (2UL << (6 * 2)) | (2UL << (7 * 2));

    /* PB0 = NRESET (Out, high), PB1 = BUSY (In), PB2 = DIO1 (In), PB4/PB5/PB10 = Out */
    GPIOB_BSRR = (1UL << 0);
    GPIOB_MODER = (GPIOB_MODER & ~((3UL << 0) | (3UL << 2) | (3UL << 4)
                                 | (3UL << 8) | (3UL << 10) | (3UL << 20)))
                | (1UL << 0) | (1UL << 8) | (1UL << 10) | (1UL << 20);

    /* SPI1: Master, fPCLK/4 = 12 MHz, CPOL=0, CPHA=0, SSM=1, SSI=1, 8-bit FRXTH=1 */
    SPI1_CR2 = (7UL << 8) | (1UL << 12);
    SPI1_CR1 = (1UL << 2) | (1UL << 3) | (1UL << 8) | (1UL << 9) | (1UL << 6);
}

static uint8_t spi1_xfer(uint8_t tx) {
    while ((SPI1_SR & (1UL << 1)) == 0UL) {}
    SPI1_DR8 = tx;
    while ((SPI1_SR & (1UL << 0)) == 0UL) {}
    return SPI1_DR8;
}

static void sx1262_wait_busy(void) {
    uint32_t guard = 240000UL;
    while (((GPIOB_IDR & (1UL << 1)) != 0UL) && (guard-- > 0UL)) {}
}

static void sx1262_write_cmd(uint8_t cmd, const uint8_t *args, uint32_t len) {
    sx1262_wait_busy();
    GPIOA_BSRR = (1UL << (4 + 16));
    (void)spi1_xfer(cmd);
    for (uint32_t i = 0; i < len; ++i) {
        (void)spi1_xfer(args[i]);
    }
    GPIOA_BSRR = (1UL << 4);
}

static void sx1262_set_freq_hz(uint32_t freq_hz) {
    /* RF_Freq = freq_hz * 2^25 / 32,000,000 = (freq_hz << 11) / 15625 */
    uint64_t pll = ((uint64_t)freq_hz << 25) / 32000000ULL;
    uint8_t buf[4];
    buf[0] = (uint8_t)(pll >> 24);
    buf[1] = (uint8_t)(pll >> 16);
    buf[2] = (uint8_t)(pll >> 8);
    buf[3] = (uint8_t)(pll);
    sx1262_write_cmd(SX_CMD_SET_RF_FREQUENCY, buf, 4U);
}

static int8_t sx1262_read_rssi_dbm(void) {
    sx1262_wait_busy();
    GPIOA_BSRR = (1UL << (4 + 16));
    (void)spi1_xfer(SX_CMD_GET_RSSI_INST);
    (void)spi1_xfer(0x00U); /* Status byte */
    uint8_t raw = spi1_xfer(0x00U);
    GPIOA_BSRR = (1UL << 4);
    return (int8_t)(-((int16_t)raw / 2));
}

static void sx1262_init_lora_915(void) {
    /* Hardware reset pulse > 100 us */
    GPIOB_BSRR = (1UL << (0 + 16));
    delay_cycles(8000U);
    GPIOB_BSRR = (1UL << 0);
    delay_cycles(48000U);

    uint8_t b4[8];
    b4[0] = 0x00U; /* STDBY_RC */
    sx1262_write_cmd(SX_CMD_SET_STANDBY, b4, 1U);

    b4[0] = 0x01U; /* USE_DCDC synchronous buck with 15 uH L1 */
    sx1262_write_cmd(SX_CMD_SET_REGULATOR_MODE, b4, 1U);

    b4[0] = 0x01U; /* DIO2 autonomous TX RF switch V1 control */
    sx1262_write_cmd(SX_CMD_SET_DIO2_RF_SW, b4, 1U);

    b4[0] = 0xE1U; b4[1] = 0xE9U; /* CalibrateImage 902..928 MHz ISM */
    sx1262_write_cmd(SX_CMD_CALIBRATE_IMAGE, b4, 2U);

    b4[0] = 0x01U; /* PACKET_TYPE_LORA */
    sx1262_write_cmd(SX_CMD_SET_PACKET_TYPE, b4, 1U);

    sx1262_set_freq_hz(906875000UL); /* US915 LongFast Meshtastic default */

    /* +22 dBm optimal PA config: paDutyCycle=0x04, hpMax=0x07, deviceSel=0x00, paLut=0x01 */
    b4[0] = 0x04U; b4[1] = 0x07U; b4[2] = 0x00U; b4[3] = 0x01U;
    sx1262_write_cmd(SX_CMD_SET_PA_CONFIG, b4, 4U);

    b4[0] = 22U; b4[1] = 0x04U; /* +22 dBm, 200 us ramp */
    sx1262_write_cmd(SX_CMD_SET_TX_PARAMS, b4, 2U);

    /* SF11, BW 250 kHz (0x05), CR 4/5 (0x01), LDRO=1 */
    b4[0] = 11U; b4[1] = 0x05U; b4[2] = 0x01U; b4[3] = 0x01U;
    sx1262_write_cmd(SX_CMD_SET_MOD_PARAMS, b4, 4U);

    /* Preamble=16, Explicit, MaxLen=255, CRC On, Standard IQ */
    b4[0] = 0x00U; b4[1] = 16U; b4[2] = 0x00U; b4[3] = 255U; b4[4] = 0x01U; b4[5] = 0x00U;
    sx1262_write_cmd(SX_CMD_SET_PKT_PARAMS, b4, 6U);

    /* Enable TxDone (bit 0) + RxDone (bit 1) + Timeout (bit 9) on DIO1 */
    b4[0] = 0x02U; b4[1] = 0x03U; b4[2] = 0x02U; b4[3] = 0x03U;
    b4[4] = 0x00U; b4[5] = 0x00U; b4[6] = 0x00U; b4[7] = 0x00U;
    sx1262_write_cmd(SX_CMD_SET_DIO_IRQ_PARAMS, b4, 8U);

    /* Assert AS179-92LF V2 RX enable & enter continuous LoRa RX */
    GPIOB_BSRR = (1UL << 10) | (1UL << 4);
    b4[0] = 0xFFU; b4[1] = 0xFFU; b4[2] = 0xFFU;
    sx1262_write_cmd(SX_CMD_SET_RX, b4, 3U);
}

static void sweep_902_928_mhz_once(void) {
    uint8_t rx_cont[3];
    rx_cont[0] = 0xFFU; rx_cont[1] = 0xFFU; rx_cont[2] = 0xFFU;
    GPIOB_BSRR = (1UL << 10) | (1UL << 4);

    /* Step across 902.0 .. 927.4 MHz in 200 kHz increments (128 bins) */
    for (uint32_t bin = 0; bin < 128U; ++bin) {
        sx1262_set_freq_hz(902000000UL + bin * 200000UL);
        sx1262_write_cmd(SX_CMD_SET_RX, rx_cont, 3U);
        delay_cycles(9600U); /* ~200 us LNA/AGC settling */
        g_waterfall_dbm[bin] = sx1262_read_rssi_dbm();
    }
}

int main(void) {
    clock_usb_init();
    gpio_spi1_init();
    sx1262_init_lora_915();

    uint32_t loop = 0;
    for (;;) {
        if (g_sweep_mode) {
            sweep_902_928_mhz_once();
        } else if ((GPIOB_IDR & (1UL << 2)) != 0UL) {
            /* Clear DIO1 IRQ status & pulse activity LED */
            uint8_t clr[2];
            clr[0] = 0xFFU; clr[1] = 0xFFU;
            sx1262_write_cmd(SX_CMD_CLR_IRQ_STATUS, clr, 2U);
            GPIOB_BSRR = (1UL << 5);
            delay_cycles(48000U);
            GPIOB_BSRR = (1UL << (5 + 16));
        }
        if ((++loop & 0x3FFFFUL) == 0UL) {
            GPIOB_BSRR = (GPIOB_IDR & (1UL << 4)) ? (1UL << (4 + 16)) : (1UL << 4);
        }
    }
}
