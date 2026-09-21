/**
 * @file  bno085.h
 * @brief BNO085 IMU driver — UART-RVC mode.
 *
 * In UART-RVC mode the sensor streams a fixed 19-byte frame at 100 Hz:
 *   AA AA | idx | yaw(2) pitch(2) roll(2) | ax(2) ay(2) az(2) | rsv(3) | cksum
 * Angles are int16 little-endian in units of 0.01 deg; accel in milli-g.
 * Output-only (sensor -> host), so only the sensor's TX wire is used.
 */
#ifndef BNO085_H
#define BNO085_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
  int16_t yaw;    /* 0.01 deg  (deg = yaw/100)  */
  int16_t pitch;  /* 0.01 deg                   */
  int16_t roll;   /* 0.01 deg                   */
  int16_t acc_x;  /* milli-g                    */
  int16_t acc_y;  /* milli-g                    */
  int16_t acc_z;  /* milli-g                    */
  uint8_t index;  /* frame sequence counter     */
} bno085_frame_t;

/** Bind to an initialized UART and start DMA reception. */
void bno085_init(UART_HandleTypeDef *huart);

/** Call from the IMU UART's RX-event (DMA idle) and error interrupts. */
void bno085_rx_event(uint16_t size);
void bno085_error_isr(void);

/** Call from the main loop; returns true and fills *out on a valid frame. */
bool bno085_process(bno085_frame_t *out);

#endif /* BNO085_H */
