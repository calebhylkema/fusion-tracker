/**
 * @file    ld2450.h
 * @brief   Driver for the HLK-LD2450 24 GHz multi-target tracking radar.
 *
 * The module streams a fixed 30-byte frame at 256000 baud, 8N1:
 *   AA FF 03 00 | 3 x [Xlo Xhi Ylo Yhi Slo Shi Rlo Rhi] | 55 CC
 * Coordinates/speed are sign-magnitude (top bit = sign, 1 = positive).
 * X/Y are mm (X = left/right of boresight, Y = distance out), speed cm/s.
 */
#ifndef LD2450_H
#define LD2450_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define LD2450_MAX_TARGETS 3

typedef struct {
  int16_t  x;          /* mm   — +/- = right/left of boresight        */
  int16_t  y;          /* mm   — distance straight out from the sensor */
  int16_t  speed;      /* cm/s — + moving away / - approaching         */
  uint16_t resolution; /* mm   — reported distance resolution          */
  uint16_t distance;   /* mm   — sqrt(x^2 + y^2), range to target      */
  bool     valid;      /* true when this slot holds a real target      */
} ld2450_target_t;

typedef struct {
  ld2450_target_t target[LD2450_MAX_TARGETS];
} ld2450_frame_t;

/** Bind the driver to an initialized UART and start reception. */
void ld2450_init(UART_HandleTypeDef *huart);

/** Call from the radar UART's RX-event (DMA idle) and error interrupts. */
void ld2450_rx_event(uint16_t size);
void ld2450_error_isr(void);

/** Call from the main loop. Returns true and fills *out once a complete,
 *  tail-validated frame has been parsed. Call in a while() to drain all. */
bool ld2450_process(ld2450_frame_t *out);

#endif /* LD2450_H */
