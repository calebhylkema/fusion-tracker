/**
 * @file  can_bus.h
 * @brief Thin wrapper over STM32 bxCAN (CAN1) for the fused-track output bus.
 *
 * Configured for 500 kbit/s. Starts in loopback mode (self-test, no wiring);
 * switch the .ioc Operating Mode to Normal for a real bus with a transceiver.
 */
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/** Configure an accept-all filter and start the CAN peripheral. */
void can_bus_init(CAN_HandleTypeDef *hcan);

/** Queue a standard-ID data frame for transmission. Returns false if no mailbox. */
bool can_bus_send(uint32_t std_id, const uint8_t *data, uint8_t len);

/** Non-blocking receive from FIFO0. Returns true and fills id/data/len if a
 *  frame is waiting; data must point to an 8-byte buffer. */
bool can_bus_poll(uint32_t *std_id, uint8_t *data, uint8_t *len);

#endif /* CAN_BUS_H */
