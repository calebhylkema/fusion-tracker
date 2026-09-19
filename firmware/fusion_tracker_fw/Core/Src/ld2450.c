/**
 * @file  ld2450.c
 * @brief HLK-LD2450 radar driver — interrupt RX into a ring buffer,
 *        header-synced frame parser, decoded target output.
 */
#include "ld2450.h"

#define LD2450_RB_SZ 512   /* ring buffer; >> one 30-byte frame */

/* --- receive plumbing (owned by the driver) --- */
static UART_HandleTypeDef *s_uart;
static volatile uint8_t    s_rb[LD2450_RB_SZ];
static volatile uint16_t   s_head;
static uint16_t            s_tail;
static uint8_t             s_rx_byte;

void ld2450_init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  s_head = 0;
  s_tail = 0;
  HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);   /* arm first byte */
}

void ld2450_rx_isr(void)
{
  s_rb[s_head] = s_rx_byte;
  s_head = (uint16_t)((s_head + 1) % LD2450_RB_SZ);
  HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);   /* re-arm next byte */
}

void ld2450_error_isr(void)
{
  HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);   /* recover from overrun */
}

/* --- decode helpers --- */

/* Sign-magnitude: top bit of the 16-bit value is the sign (1 = positive). */
static int16_t decode(uint8_t lo, uint8_t hi)
{
  int16_t mag = (int16_t)(((hi & 0x7F) << 8) | lo);
  return (hi & 0x80) ? mag : (int16_t)(-mag);
}

/* Integer sqrt — keeps libm out of the firmware. */
static uint16_t isqrt(uint32_t n)
{
  uint32_t res = 0, bit = 1UL << 30;
  while (bit > n) bit >>= 2;
  while (bit) {
    if (n >= res + bit) { n -= res + bit; res = (res >> 1) + bit; }
    else                  res >>= 1;
    bit >>= 2;
  }
  return (uint16_t)res;
}

static void decode_frame(const uint8_t *d, ld2450_frame_t *f)
{
  for (int t = 0; t < LD2450_MAX_TARGETS; t++) {
    const uint8_t   *p  = &d[t * 8];
    ld2450_target_t *tg = &f->target[t];
    tg->x          = decode(p[0], p[1]);
    tg->y          = decode(p[2], p[3]);
    tg->speed      = decode(p[4], p[5]);
    tg->resolution = (uint16_t)(p[6] | (p[7] << 8));
    tg->distance   = isqrt((uint32_t)((int32_t)tg->x * tg->x + (int32_t)tg->y * tg->y));
    tg->valid      = (tg->resolution != 0);
  }
}

/* --- header-synced frame parser (state persists across calls) --- */
bool ld2450_process(ld2450_frame_t *out)
{
  static const uint8_t HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
  static uint8_t hdr = 0;      /* header bytes matched so far   */
  static uint8_t buf[26];      /* 24 target bytes + 2 tail bytes */
  static uint8_t idx = 0;
  static uint8_t in_frame = 0;

  while (s_tail != s_head) {
    uint8_t b = s_rb[s_tail];
    s_tail = (uint16_t)((s_tail + 1) % LD2450_RB_SZ);

    if (!in_frame) {
      if (b == HDR[hdr]) {
        if (++hdr == 4) { hdr = 0; idx = 0; in_frame = 1; }
      } else {
        hdr = (b == HDR[0]) ? 1 : 0;   /* allow immediate re-sync on 0xAA */
      }
      continue;
    }

    buf[idx++] = b;
    if (idx == 26) {
      in_frame = 0;
      if (buf[24] == 0x55 && buf[25] == 0xCC) {   /* valid tail */
        decode_frame(buf, out);
        return true;                              /* one frame per call */
      }
    }
  }
  return false;
}
