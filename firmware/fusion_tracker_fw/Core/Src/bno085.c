/**
 * @file  bno085.c
 * @brief BNO085 UART-RVC driver — DMA + idle-line RX, 19-byte frame parser.
 */
#include "bno085.h"

#define BNO_RB_SZ 256

static UART_HandleTypeDef *s_uart;
static volatile uint8_t    s_rb[BNO_RB_SZ];
static volatile uint16_t   s_head;
static uint16_t            s_tail;
static uint8_t             s_dma_buf[64];

void bno085_init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  s_head = 0;
  s_tail = 0;
  HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_dma_buf, sizeof(s_dma_buf));
  __HAL_DMA_DISABLE_IT(s_uart->hdmarx, DMA_IT_HT);
}

void bno085_rx_event(uint16_t size)
{
  for (uint16_t i = 0; i < size; i++) {
    s_rb[s_head] = s_dma_buf[i];
    s_head = (uint16_t)((s_head + 1) % BNO_RB_SZ);
  }
  HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_dma_buf, sizeof(s_dma_buf));
  __HAL_DMA_DISABLE_IT(s_uart->hdmarx, DMA_IT_HT);
}

void bno085_error_isr(void)
{
  /* Clear the error (ORE/noise) and fully restart, else RX can stall forever. */
  HAL_UART_AbortReceive(s_uart);
  __HAL_UART_CLEAR_OREFLAG(s_uart);
  HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_dma_buf, sizeof(s_dma_buf));
  __HAL_DMA_DISABLE_IT(s_uart->hdmarx, DMA_IT_HT);
}

static int16_t le16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

/* Frame after the two 0xAA header bytes = 17 bytes: idx..checksum.
 * Checksum = (sum of the 16 bytes idx..reserved) & 0xFF. */
bool bno085_process(bno085_frame_t *out)
{
  static uint8_t hdr = 0;      /* consecutive 0xAA header bytes seen */
  static uint8_t buf[17];      /* idx(1)+angles(6)+accel(6)+rsv(3)+cksum(1) */
  static uint8_t idx = 0;
  static uint8_t in_frame = 0;

  while (s_tail != s_head) {
    uint8_t b = s_rb[s_tail];
    s_tail = (uint16_t)((s_tail + 1) % BNO_RB_SZ);

    if (!in_frame) {
      if (b == 0xAA) {
        if (++hdr == 2) { hdr = 0; idx = 0; in_frame = 1; }
      } else {
        hdr = 0;
      }
      continue;
    }

    buf[idx++] = b;
    if (idx == 17) {
      in_frame = 0;
      uint8_t sum = 0;
      for (int i = 0; i < 16; i++) sum += buf[i];
      if (sum == buf[16]) {                 /* checksum ok */
        out->index = buf[0];
        out->yaw   = le16(&buf[1]);
        out->pitch = le16(&buf[3]);
        out->roll  = le16(&buf[5]);
        out->acc_x = le16(&buf[7]);
        out->acc_y = le16(&buf[9]);
        out->acc_z = le16(&buf[11]);
        return true;
      }
    }
  }
  return false;
}
