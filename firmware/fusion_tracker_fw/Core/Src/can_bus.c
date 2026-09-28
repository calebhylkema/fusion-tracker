/**
 * @file  can_bus.c
 * @brief STM32 bxCAN wrapper — accept-all filter, TX/RX helpers.
 */
#include "can_bus.h"

static CAN_HandleTypeDef *s_hcan;

void can_bus_init(CAN_HandleTypeDef *hcan)
{
  s_hcan = hcan;

  /* Accept every standard/extended ID into RX FIFO0 (mask = 0). */
  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh = 0;
  f.FilterIdLow = 0;
  f.FilterMaskIdHigh = 0;
  f.FilterMaskIdLow = 0;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(s_hcan, &f);

  HAL_CAN_Start(s_hcan);
}

bool can_bus_send(uint32_t std_id, const uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef tx = {0};
  tx.StdId = std_id;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = (len > 8) ? 8 : len;
  uint32_t mailbox;
  return HAL_CAN_AddTxMessage(s_hcan, &tx, (uint8_t *)data, &mailbox) == HAL_OK;
}

bool can_bus_poll(uint32_t *std_id, uint8_t *data, uint8_t *len)
{
  if (HAL_CAN_GetRxFifoFillLevel(s_hcan, CAN_RX_FIFO0) == 0)
    return false;
  CAN_RxHeaderTypeDef rx;
  if (HAL_CAN_GetRxMessage(s_hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK)
    return false;
  *std_id = rx.StdId;
  *len = (uint8_t)rx.DLC;
  return true;
}
