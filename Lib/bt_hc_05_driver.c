/*
 * bt_hc_05_driver.c
 *
 *  Created on: Jan 28, 2022
 *      Author: Morgan
 */

#include "bt_hc_05_driver.h"
#include "stm32f407xx.h"

// TX_MCU =

#define BT_HC_05_RX_BUF_SIZE 256
#define BT_HC_05_GPIO_STATE_PORT GPIOE
#define BT_HC_05_GPIO_STATE_PIN GPIO_PIN_7

#define BT_HC_05_GPIO_EN_PORT GPIOB
#define BT_HC_05_GPIO_EN_PIN GPIO_PIN_1

#define BT_HC_05_SET_EN_PIN() \
	HAL_GPIO_WritePin(BT_HC_05_GPIO_STATE_PORT, BT_HC_05_GPIO_EN_PIN, GPIO_PIN_SET)

#define BT_HC_05_RESET_EN_PIN() \
	HAL_GPIO_WritePin(BT_HC_05_GPIO_STATE_PORT, BT_HC_05_GPIO_EN_PIN, GPIO_PIN_RESET)

bool isAtModeEnabled = false;

uint8_t btBuffer[BT_HC_05_RX_BUF_SIZE] = { 0 };

static struct
{
	UART_HandleTypeDef *uartHandler;
	DMA_HandleTypeDef *dmaUartRx;
	DMA_HandleTypeDef *dmaUartTx;

}btHc05Uart;

void bt_hc_05_init(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx, DMA_HandleTypeDef *dmaUartTx)
{
	btHc05Uart.uartHandler = uartHandler;
	btHc05Uart.dmaUartRx = dmaUartRx;
	btHc05Uart.dmaUartTx = dmaUartTx;
}

void bt_hc_05_switch_device_mode(bool isGoToAtMode)
{
	if (isGoToAtMode)
	{
		BT_HC_05_SET_EN_PIN();
	}
	else
	{
		BT_HC_05_RESET_EN_PIN();
	}
	isAtModeEnabled = isGoToAtMode;
}

void bt_hc_05_read_data()
{
	 HAL_UART_Receive_DMA(btHc05Uart.uartHandler, btBuffer, BT_HC_05_RX_BUF_SIZE);
}












