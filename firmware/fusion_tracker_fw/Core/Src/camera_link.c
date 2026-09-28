/**
 * @file  camera_link.c
 * @brief Pi -> STM32 camera-detection receiver (USART6, DMA + idle-line).
 */
#include "camera_link.h"

#define CAM_RB_SZ 128
#define DEG2RAD (0.017453293f)   /* pi/180 */

static UART_HandleTypeDef *s_uart;
static volatile uint8_t    s_rb[CAM_RB_SZ];
static volatile uint16_t   s_head;
static uint16_t            s_tail;
static uint8_t             s_dma[32];
static volatile uint32_t   s_bytes = 0;   /* total raw bytes received (diagnostic) */

void camera_init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  s_head = 0;
  s_tail = 0;
  HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_dma, sizeof(s_dma));
  __HAL_DMA_DISABLE_IT(s_uart->hdmarx, DMA_IT_HT);
}

uint32_t camera_rx_bytes(void) { return s_bytes; }

void camera_rx_event(uint16_t size)
{
  s_bytes += size;
  for (uint16_t i = 0; i < size; i++) {
    s_rb[s_head] = s_dma[i];
    s_head = (uint16_t)((s_head + 1) % CAM_RB_SZ);
  }
  HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_dma, sizeof(s_dma));
  __HAL_DMA_DISABLE_IT(s_uart->hdmarx, DMA_IT_HT);
}

void camera_error_isr(void)
{
  /* A single overrun/noise glitch otherwise stalls ReceiveToIdle_DMA forever.
   * Abort, clear the error flags (ORE etc.), then fully restart. */
  HAL_UART_AbortReceive(s_uart);
  __HAL_UART_CLEAR_OREFLAG(s_uart);
  HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_dma, sizeof(s_dma));
  __HAL_DMA_DISABLE_IT(s_uart->hdmarx, DMA_IT_HT);
}

/* Frame: A5 5A | blo bhi flags cksum  (payload = 4 bytes after the header). */
bool camera_process(camera_det_t *out)
{
  static uint8_t st = 0;      /* 0=wait A5, 1=wait 5A, 2=collect payload */
  static uint8_t buf[4];
  static uint8_t idx = 0;

  while (s_tail != s_head) {
    uint8_t b = s_rb[s_tail];
    s_tail = (uint16_t)((s_tail + 1) % CAM_RB_SZ);

    if (st == 0) {
      if (b == 0xA5) st = 1;
    } else if (st == 1) {
      st = (b == 0x5A) ? 2 : ((b == 0xA5) ? 1 : 0);
      idx = 0;
    } else {
      buf[idx++] = b;
      if (idx == 4) {
        st = 0;
        if (((buf[0] + buf[1] + buf[2]) & 0xFF) == buf[3]) {     /* checksum ok */
          int16_t cd = (int16_t)(buf[0] | (buf[1] << 8));
          out->bearing_rad = cd * 0.01f * DEG2RAD;              /* centideg -> rad */
          out->present = (buf[2] & 0x01);
          return true;
        }
      }
    }
  }
  return false;
}
