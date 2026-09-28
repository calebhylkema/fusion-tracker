/**
 * @file  camera_link.h
 * @brief Receives camera detections from the Raspberry Pi over UART (USART6).
 *
 * Pi frame (6 bytes): A5 5A | bearing_lo bearing_hi | flags | checksum
 *   bearing : int16 LE, centi-degrees (deg*100), + = right of boresight
 *   flags   : bit0 = person present
 *   checksum: (sum of the 3 payload bytes) & 0xFF
 */
#ifndef CAMERA_LINK_H
#define CAMERA_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
  float bearing_rad;   /* azimuth of the person, rad, + = right of boresight */
  bool  present;       /* a person was detected                             */
} camera_det_t;

void camera_init(UART_HandleTypeDef *huart);
void camera_rx_event(uint16_t size);   /* from HAL_UARTEx_RxEventCallback */
void camera_error_isr(void);           /* from HAL_UART_ErrorCallback     */
bool camera_process(camera_det_t *out);/* main loop; true on a valid frame */

#endif /* CAMERA_LINK_H */
