#ifndef BLUETOOTH_BT_STATE_MACHINE_H_
#define BLUETOOTH_BT_STATE_MACHINE_H_

#include "stdbool.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

enum BtState
{
	BT_STATE_NOT_INITIALIZED = 0,
	BT_STATE_INITIALIZING,
	BT_STATE_AT_MODE_ENABLING,
	BT_STATE_SCANINIG,
	BT_STATE_DATA_MODE_ENABLING,
	// Под BT_STATE_DATA_MODE подразумевается режим получения данных от ЭЭГ
	BT_STATE_ESTABLISHING_CONNECTION_WITH_CHILD,
	BT_STATE_PROCESSING_DATA_FROM_CHILD,
	// Под BT_STATE_CONFIG_MODE подразумевается конфигурационный BT-режим.
	BT_STATE_CONFIG_MODE,
	BT_STATE_RESTART_MODULE
};

extern uint32_t btState;

bool bt_state_machine_process_states(uint32_t *delayBeforeNextUpdateMs, bool needToWaitForConfiguration);
void bt_state_machine_start(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx);


#endif /* BLUETOOTH_BT_STATE_MACHINE_H_ */
