/*
 * Day 07 — Polyphonic FM / Chiptune I2S Synthesizer & Type-A MIDI Expansion Card
 * Target MCU: STM32F072CBT6 (48 MHz Cortex-M0, Crystal-less HSI48 + CRS USB-SOF Trim)
 *
 * Hardware Mapping (matches Day_07_Chiptune_Synth_MIDI.kicad_sch):
 *   PA0..PA3           : VOICE_LED1..4 (4-Channel Chiptune Gate / Envelope Bar)
 *   PA9  (USART1_TX)   : 31,250 baud 8N1 MMA Type-A Hardware MIDI Out (via 22R R9/R10 -> J3)
 *   PA11 / PA12        : USB 2.0 FS Device (USB-MIDI 1.0 + CRS 1 kHz SOF clock recovery)
 *   PB12 (I2S2_WS)     : I2S_LRCK -> PCM5102APWR Pin 15 (48 kHz Stereo Frame Sync)
 *   PB13 (I2S2_CK)     : I2S_BCK  -> PCM5102APWR Pin 13 (1.536 MHz Bit Clock / Internal PLL)
 *   PB14 (GPIO Out)    : DAC_XSMT -> PCM5102APWR Pin 17 (Pop-Free Soft-Mute Release)
 *   PB15 (I2S2_SD)     : I2S_DIN  -> PCM5102APWR Pin 14 (16-bit Stereo I2S Audio Stream)
 */

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))
#define REG16(addr) (*(volatile uint16_t *)(addr))

/* Base Peripheral Addresses (STM32F072xB) */
#define RCC_BASE        0x40021000UL
#define FLASH_BASE_R    0x40022000UL
#define CRS_BASE        0x40006C00UL
#define GPIOA_BASE      0x48000000UL
#define GPIOB_BASE      0x48000400UL
#define SPI2_BASE       0x40003800UL
#define USART1_BASE     0x40013800UL
#define USB_BASE        0x40005C00UL

/* RCC Registers */
#define RCC_CR          REG32(RCC_BASE + 0x00UL)
#define RCC_CFGR        REG32(RCC_BASE + 0x04UL)
#define RCC_APB2ENR     REG32(RCC_BASE + 0x18UL)
#define RCC_APB1ENR     REG32(RCC_BASE + 0x1CUL)
#define RCC_AHBENR      REG32(RCC_BASE + 0x14UL)
#define RCC_CR2         REG32(RCC_BASE + 0x34UL)
#define FLASH_ACR       REG32(FLASH_BASE_R + 0x00UL)

/* CRS Registers (USB-SOF Automatic HSI48 Trimming) */
#define CRS_CR          REG32(CRS_BASE + 0x00UL)
#define CRS_CFGR        REG32(CRS_BASE + 0x04UL)

/* GPIO Registers */
#define GPIOA_MODER     REG32(GPIOA_BASE + 0x00UL)
#define GPIOA_AFRH      REG32(GPIOA_BASE + 0x24UL)
#define GPIOA_BSRR      REG32(GPIOA_BASE + 0x18UL)

#define GPIOB_MODER     REG32(GPIOB_BASE + 0x00UL)
#define GPIOB_AFRH      REG32(GPIOB_BASE + 0x24UL)
#define GPIOB_BSRR      REG32(GPIOB_BASE + 0x18UL)

/* SPI2 / I2S2 Registers */
#define SPI2_SR         REG16(SPI2_BASE + 0x08UL)
#define SPI2_DR         REG16(SPI2_BASE + 0x0CUL)
#define SPI2_I2SCFGR    REG16(SPI2_BASE + 0x1CUL)
#define SPI2_I2SPR      REG16(SPI2_BASE + 0x20UL)

/* USART1 Registers (31.25 kbaud MIDI Out) */
#define USART1_CR1      REG32(USART1_BASE + 0x00UL)
#define USART1_BRR      REG32(USART1_BASE + 0x0CUL)
#define USART1_ISR      REG32(USART1_BASE + 0x1CUL)
#define USART1_TDR      REG32(USART1_BASE + 0x28UL)

/* USB FS Device Registers */
#define USB_ISTR        REG16(USB_BASE + 0x44UL)
#define USB_CNTR        REG16(USB_BASE + 0x40UL)
#define USB_BCDR        REG16(USB_BASE + 0x58UL)

typedef struct {
    uint32_t phase;
    uint32_t phase_inc;
    uint32_t mod_phase;
    uint16_t env;         /* Q15 amplitude envelope */
    uint8_t  waveform;    /* 0: 2-Op FM, 1: 25% Pulse, 2: Triangle, 3: 15-bit LFSR Noise */
    uint8_t  note;
} synth_voice_t;

static synth_voice_t g_voices[4];
static uint16_t g_lfsr = 0x7FFFU;

/* Quarter-wave 64-entry 8-bit sine LUT for 2-Operator FM & Triangle/Sine blend */
static const int8_t k_sin64[64] = {
      0,   3,   6,   9,  12,  15,  18,  21,  24,  27,  30,  33,  36,  39,  42,  45,
     48,  51,  54,  57,  59,  62,  65,  67,  70,  73,  75,  78,  80,  82,  85,  87,
     89,  91,  94,  96,  98, 100, 102, 103, 105, 107, 108, 110, 112, 113, 114, 116,
    117, 118, 119, 120, 121, 122, 123, 123, 124, 125, 125, 126, 126, 126, 126, 127
};

static int16_t fast_sin(uint8_t angle) {
    uint8_t idx = angle & 0x3FU;
    if (angle & 0x40U) {
        idx = (uint8_t)(63U - idx);
    }
    int16_t s = k_sin64[idx];
    return (angle & 0x80U) ? (int16_t)(-s) : s;
}

/* Phase increments at Fs = 48,000 Hz for C-minor chiptune arpeggio demo / MIDI notes */
static const uint32_t k_arp_inc[8] = {
    23405858UL, /* C4  = 261.63 Hz */
    27834386UL, /* Eb4 = 311.13 Hz */
    35069236UL, /* G4  = 392.00 Hz */
    41704154UL, /* Bb4 = 466.16 Hz */
    46811716UL, /* C5  = 523.25 Hz */
    55668772UL, /* Eb5 = 622.25 Hz */
    70138472UL, /* G5  = 783.99 Hz */
    93623432UL  /* C6  = 1046.50 Hz */
};

static void clock_init_hsi48_crs(void) {
    FLASH_ACR = 0x11UL;              /* 1 wait state + Prefetch for 48 MHz */
    RCC_CR2 |= (1UL << 16);          /* Enable internal 48 MHz HSI48 RC oscillator */
    while ((RCC_CR2 & (1UL << 17)) == 0UL) {}

    RCC_CFGR = (RCC_CFGR & ~0x3UL) | 0x3UL; /* SW = 11: HSI48 as SYSCLK */
    while (((RCC_CFGR >> 2) & 0x3UL) != 0x3UL) {}

    /* Enable CRS clock and lock HSI48 trim to 1 kHz USB SOF frames */
    RCC_APB1ENR |= (1UL << 27) | (1UL << 23) | (1UL << 14); /* CRS + USB + SPI2 */
    RCC_APB2ENR |= (1UL << 14);                             /* USART1 */
    RCC_AHBENR  |= (1UL << 17) | (1UL << 18);               /* GPIOA + GPIOB */

    CRS_CR |= (1UL << 6) | (1UL << 5); /* AUTOTRIMEN + CEN */
}

static void midi_uart_init(void) {
    /* PA9 -> AF1 (USART1_TX), 48 MHz / 31,250 baud = 1536 */
    GPIOA_MODER = (GPIOA_MODER & ~(3UL << 18)) | (2UL << 18);
    GPIOA_AFRH  = (GPIOA_AFRH  & ~(0xFUL << 4)) | (1UL << 4);

    USART1_BRR = 1536UL;
    USART1_CR1 = (1UL << 3) | (1UL << 0); /* TE + UE */
}

static void midi_send_byte(uint8_t b) {
    while ((USART1_ISR & (1UL << 7)) == 0UL) {}
    USART1_TDR = b;
}

static void midi_note_on(uint8_t ch, uint8_t note, uint8_t vel) {
    midi_send_byte((uint8_t)(0x90U | (ch & 0x0FU)));
    midi_send_byte((uint8_t)(note & 0x7FU));
    midi_send_byte((uint8_t)(vel & 0x7FU));
}

static void i2s2_pcm5102_init(void) {
    /* PA0..PA3 -> General-purpose outputs for 4-Voice Bar LEDs (D3..D6) */
    GPIOA_MODER = (GPIOA_MODER & ~0xFFUL) | 0x55UL;

    /* PB12 (WS=AF0), PB13 (CK=AF0), PB15 (SD=AF0); PB14 = Output (DAC_XSMT) */
    GPIOB_MODER = (GPIOB_MODER & ~0xFF000000UL) | 0x9A000000UL;
    GPIOB_AFRH  &= ~0xF0FF0000UL;

    /* Hold XSMT low during I2S clock startup so PCM5102A PLL locks pop-free */
    GPIOB_BSRR = (1UL << (14 + 16));

    /* I2S2 Master Transmit, Philips I2S standard, 16-bit data in 16-bit frame:
     * Fs = 48 MHz / (32 * 2 * I2SDIV + ODD) -> I2SDIV = 15, ODD = 1 => 48,000.0 Hz!
     */
    SPI2_I2SPR   = (1U << 8) | 15U;
    SPI2_I2SCFGR = (1U << 11) | (2U << 8) | (1U << 10); /* I2SMOD + Master TX + I2SE */

    /* Enable internal USB FS pull-up (BCDR DPPU) for USB-MIDI enumeration & SOF sync */
    USB_CNTR = 0x0000U;
    USB_BCDR = (1U << 15);
}

static int16_t render_voice(synth_voice_t *v) {
    if (v->env == 0U) {
        return 0;
    }
    v->phase += v->phase_inc;
    v->mod_phase += (v->phase_inc << 1); /* 2:1 harmonic FM modulator */
    uint8_t p = (uint8_t)(v->phase >> 24);
    int16_t raw = 0;

    switch (v->waveform) {
        case 0: { /* 2-Operator FM Bell / Lead */
            int16_t mod = fast_sin((uint8_t)(v->mod_phase >> 24));
            uint8_t fm_ang = (uint8_t)(p + (mod >> 2));
            raw = (int16_t)(fast_sin(fm_ang) * 96);
            break;
        }
        case 1: { /* Classic 25% Duty Pulse Wave */
            raw = (p < 64U) ? 9500 : -3200;
            break;
        }
        case 2: { /* 4-Bit Quantized NES-Style Triangle Bass */
            uint8_t tri = (p & 0x80U) ? (uint8_t)(255U - p) : p;
            raw = (int16_t)(((int16_t)(tri & 0x78U) - 60) * 160);
            break;
        }
        default: { /* 15-Bit NES/GameBoy LFSR Percussion */
            if ((p & 0x07U) == 0U) {
                uint16_t fb = (uint16_t)(((g_lfsr >> 0) ^ (g_lfsr >> 1)) & 1U);
                g_lfsr = (uint16_t)((g_lfsr >> 1) | (fb << 14));
            }
            raw = (g_lfsr & 1U) ? 7000 : -7000;
            break;
        }
    }
    return (int16_t)(((int32_t)raw * (int32_t)v->env) >> 15);
}

static void update_voice_leds(void) {
    uint32_t set_mask = 0UL;
    uint32_t clr_mask = 0UL;
    for (uint32_t i = 0; i < 4U; i++) {
        if (g_voices[i].env > 4096U) {
            set_mask |= (1UL << i);
        } else {
            clr_mask |= (1UL << (i + 16U));
        }
    }
    GPIOA_BSRR = set_mask | clr_mask;
}

int main(void) {
    clock_init_hsi48_crs();
    midi_uart_init();
    i2s2_pcm5102_init();

    g_voices[0].waveform = 0U; /* Voice 1: 2-Op FM Lead */
    g_voices[1].waveform = 1U; /* Voice 2: 25% Pulse Harmony */
    g_voices[2].waveform = 2U; /* Voice 3: Quantized Triangle Bass */
    g_voices[3].waveform = 3U; /* Voice 4: LFSR Chiptune Snare/Hat */

    uint32_t sample_idx = 0UL;
    uint16_t step_timer = 0U;
    uint8_t  step_idx   = 0U;

    while (1) {
        /* Unmute PCM5102APWR after 2,048 quiet I2S frames (~42 ms PLL settle) */
        if (sample_idx == 2048UL) {
            GPIOB_BSRR = (1UL << 14);
        }

        /* Trigger polyphonic chiptune pattern at 150 BPM (6,000 samples / 16th-note) */
        if (step_timer == 0U) {
            step_timer = 5999U;
            uint8_t arp = step_idx & 7U;
            g_voices[0].phase_inc = k_arp_inc[arp];
            g_voices[0].env       = 28000U;

            if ((step_idx & 1U) == 0U) {
                g_voices[1].phase_inc = k_arp_inc[(arp + 2U) & 7U];
                g_voices[1].env       = 22000U;
            }
            if ((step_idx & 3U) == 0U) {
                g_voices[2].phase_inc = k_arp_inc[arp & 3U] >> 1;
                g_voices[2].env       = 30000U;
                midi_note_on(0U, (uint8_t)(60U + arp), 100U);
            }
            if ((step_idx & 1U) == 1U) {
                g_voices[3].phase_inc = 38000000UL;
                g_voices[3].env       = 18000U;
            }
            step_idx++;
        }

        /* Exponential envelope decay every 64 audio samples (750 Hz control rate) */
        if ((sample_idx & 63UL) == 0UL) {
            for (uint32_t i = 0; i < 4U; i++) {
                uint16_t e = g_voices[i].env;
                g_voices[i].env = (uint16_t)(e - (e >> ((i == 3U) ? 3U : 5U)));
            }
            update_voice_leds();
        }

        int32_t v0 = render_voice(&g_voices[0]);
        int32_t v1 = render_voice(&g_voices[1]);
        int32_t v2 = render_voice(&g_voices[2]);
        int32_t v3 = render_voice(&g_voices[3]);

        /* Wide stereo chiptune panning: Voice 1 biased L, Voice 2 biased R, Bass/Drums centered */
        int32_t left  = (v0 * 3 + v1 * 1 + v2 * 2 + v3 * 2) >> 2;
        int32_t right = (v0 * 1 + v1 * 3 + v2 * 2 + v3 * 2) >> 2;

        if (left > 32767)  left = 32767;
        if (left < -32768) left = -32768;
        if (right > 32767)  right = 32767;
        if (right < -32768) right = -32768;

        /* Stream Left then Right 16-bit stereo samples into SPI2/I2S2 TX buffer */
        while ((SPI2_SR & (1U << 1)) == 0U) {}
        SPI2_DR = (uint16_t)((int16_t)left);
        while ((SPI2_SR & (1U << 1)) == 0U) {}
        SPI2_DR = (uint16_t)((int16_t)right);

        sample_idx++;
        step_timer--;
    }
}
