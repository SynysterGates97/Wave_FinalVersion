#ifndef BT_HC_05_DRIVER_H_
#define BT_HC_05_DRIVER_H_

#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "bt_hc_05_driver.h"

void bt_hc_05_init(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx);

void bt_hc_05_switch_device_mode(bool isGoToAtMode);

bool bt_hc_05_connect_to_child_device();
bool bt_hc_05_find_child_device();

#endif /* BT_HC_05_DRIVER_H_ */
