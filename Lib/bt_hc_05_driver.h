/*
 * bt_hc_05_driver.h
 *
 *  Created on: Jan 28, 2022
 *      Author: Morgan
 */

#ifndef BT_HC_05_DRIVER_H_
#define BT_HC_05_DRIVER_H_

#include "stdbool.h"
#include "stm32f4xx_hal.h"

void bt_hc_05_init(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx);

void bt_hc_05_switch_device_mode(bool isGoToAtMode);

#endif /* BT_HC_05_DRIVER_H_ */