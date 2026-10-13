/*
 * Binary CSI frame sent over USB-Serial-JTAG. All fields little-endian.
 *
 *   csi_frame_hdr_t | int8_t csi[csi_len] | uint32_t crc32
 *
 * crc32 = standard CRC-32 (zlib / binascii.crc32) over header + csi.
 * csi is the raw esp_wifi buffer, unscaled: [imag0, real0, imag1, real1, ...].
 * Host-side parser: tools/csi_frame.py. Keep both in sync; bump version on change.
 */
#pragma once

#include <stdint.h>

#define CSI_FRAME_MAGIC0        0xC5
#define CSI_FRAME_MAGIC1        0x1A
#define CSI_FRAME_VERSION       1
#define CSI_FRAME_MAX_CSI_LEN   512     /* HE20: 256 subcarriers x (imag, real) */

#define CSI_FRAME_FLAG_FIRST_WORD_INVALID   (1 << 0)
#define CSI_FRAME_FLAG_GAIN_FORCED          (1 << 1)   /* RX gain locked (host command 'g') */
#define CSI_FRAME_FLAG_RELOCK_SHIFT         8          /* bits 8..15: PLL relock count mod 256 ('F'/'r') */

typedef struct __attribute__((packed)) {
    uint8_t  magic[2];      /* CSI_FRAME_MAGIC0, CSI_FRAME_MAGIC1 */
    uint8_t  version;       /* CSI_FRAME_VERSION */
    uint8_t  hdr_len;       /* sizeof(csi_frame_hdr_t) */
    uint16_t csi_len;       /* bytes of csi[] following the header */
    uint16_t flags;         /* CSI_FRAME_FLAG_* */
    uint32_t frame_seq;     /* RX-local counter, +1 per accepted CSI callback;
                               gaps = frames dropped on this device (queue full) */
    uint32_t tx_seq;        /* sender counter carried in the ESP-NOW payload;
                               gaps with continuous frame_seq = lost over the air */
    uint32_t timestamp_us;  /* rx_ctrl.timestamp, local receive time */
    uint8_t  mac[6];        /* source MAC */
    int8_t   rssi;          /* dBm */
    int8_t   noise_floor;   /* dBm */
    uint8_t  channel;       /* primary channel */
    uint8_t  second;        /* secondary channel */
    uint8_t  rate;          /* rx_ctrl.rate */
    uint8_t  bb_format;     /* rx_ctrl.cur_bb_format, 4 = HE SU */
    uint8_t  agc_gain;      /* raw RX gain, for host-side compensation */
    int8_t   fft_gain;
    uint8_t  rx_state;      /* 0 = success */
    int8_t   temp_c;        /* chip internal temperature, degC, updated 1 Hz */
    uint16_t sig_len;       /* MPDU length */
    uint16_t rx_seq;        /* 802.11 sequence number */
    uint32_t he_siga1;
    uint16_t he_siga2;
    uint16_t lo_mhz;        /* RF PLL frequency set by host 'F'/'r', 0 = untouched */
} csi_frame_hdr_t;

_Static_assert(sizeof(csi_frame_hdr_t) == 48, "csi_frame_hdr_t layout changed");
