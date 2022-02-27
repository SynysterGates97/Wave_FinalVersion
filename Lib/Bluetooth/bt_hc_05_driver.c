#include "bt_hc_05_driver.h"
#include "stm32f407xx.h"

// TX_MCU =

#define BT_HC_05_RX_BUF_SIZE 256
#define BT_HC_05_GPIO_STATE_PORT GPIOE
#define BT_HC_05_GPIO_STATE_PIN GPIO_PIN_7

#define BT_HC_05_GPIO_EN_PORT GPIOB
#define BT_HC_05_GPIO_EN_PIN GPIO_PIN_1

#define BT_HC_05_AT_MODE_UART_BAUD_RATE 38400

#define BT_HC_05_DATA_MODE_UART_BAUD_RATE 57600

#define BT_HC_05_SET_EN_PIN() \
	HAL_GPIO_WritePin(BT_HC_05_GPIO_EN_PORT, BT_HC_05_GPIO_EN_PIN, GPIO_PIN_SET)

#define BT_HC_05_RESET_EN_PIN() \
	HAL_GPIO_WritePin(BT_HC_05_GPIO_EN_PORT, BT_HC_05_GPIO_EN_PIN, GPIO_PIN_RESET)

#define WAVE_CHILD_DEVICE_NAME "AlphaWaveSon"

bool isAtModeEnabled = false;

uint8_t btBuffer[BT_HC_05_RX_BUF_SIZE] = { 0 };

static struct
{
	UART_HandleTypeDef *uartHandler;
	DMA_HandleTypeDef *dmaUartRx;
	DMA_HandleTypeDef *dmaUartTx;

}btHc05Uart;

void bt_hc_05_init(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx)
{
	btHc05Uart.uartHandler = uartHandler;
	btHc05Uart.dmaUartRx = dmaUartRx;
}

// TODO: когда добавлю на плату возможность отключать питание для модуля, нужно будет отключать его для выхода из режима данных.
// Сейчас программно из режима данных в режим AT-команд не вернуться!!!
// Хотя нужно ещё подумать нужно ли это. Возможно достаточно просто обойтись кнопкой отключающей питание МК.
void bt_hc_05_switch_device_mode(bool isGoToAtMode)
{
	uint32_t newUartSpeed = 0;
	if (isGoToAtMode)
	{
		newUartSpeed = BT_HC_05_AT_MODE_UART_BAUD_RATE;

		BT_HC_05_SET_EN_PIN();
	}
	else
	{
		newUartSpeed = BT_HC_05_DATA_MODE_UART_BAUD_RATE;

		BT_HC_05_RESET_EN_PIN();
		HAL_Delay(5);
		HAL_StatusTypeDef transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+RESET\r\n", 10, 3000);
	}

	btHc05Uart.uartHandler->Init.BaudRate = newUartSpeed;
	HAL_StatusTypeDef deinitResult = HAL_UART_DeInit(btHc05Uart.uartHandler);

	if (deinitResult == HAL_OK)
	{
		HAL_StatusTypeDef initResult = HAL_UART_Init(btHc05Uart.uartHandler);
		if (initResult == HAL_OK && isGoToAtMode)
		{
			HAL_StatusTypeDef transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+RESET\r\n", 10, 3000);
		}
	}

	isAtModeEnabled = isGoToAtMode;
}

void bt_hc_05_read_data()
{
	HAL_StatusTypeDef uartStat = HAL_UART_Receive_DMA(btHc05Uart.uartHandler, btBuffer, 5);
}

// TODO: Функция инициализирует подключение к дочернему устройству
// Функцию может вызываться несколько раз уже в процессе подключения, ЭТО НЕ ДОЛЖНО ЛОМАТЬ ЛОГИКУ!
bool bt_hc_05_connect_to_child_device()
{
	return true;
}

// TODO: Функция запускает поиск дочернего устройства по имени
// Функцию может вызываться несколько раз уже в процессе поиска, ЭТО НЕ ДОЛЖНО ЛОМАТЬ ЛОГИКУ!
bool bt_hc_05_find_child_device()
{
	return true;
}










