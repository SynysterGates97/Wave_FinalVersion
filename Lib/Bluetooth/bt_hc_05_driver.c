#include "bt_hc_05_driver.h"
#include "stm32f407xx.h"

#include <string.h>
#include "lcd.h"
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

#define AT_COMMAND_OK_RESPONSE "OK"

///////////////////////////
#define UART_TRANSMITION_TIMEOUT_MS 1000

bool isAtModeEnabled = false;

uint8_t btBuffer[BT_HC_05_RX_BUF_SIZE] = { 0 };

typedef void (*RxCallbackFunction)(uint8_t *, uint32_t);
typedef void (*SendDataFunction)(uint8_t *, uint32_t, bool);

static struct
{
	UART_HandleTypeDef *uartHandler;
	uint8_t *rxBuf;
	DMA_HandleTypeDef *dmaUartRx;
	DMA_HandleTypeDef *dmaUartTx;
	RxCallbackFunction rx_callback_function;
	SendDataFunction send_data_function;

	uint32_t *rx_parameter;
	uint32_t sendAtCommandTimeBeginMs;
}btHc05Uart;

void bt_hc_send_data(uint8_t *dataToSend, uint32_t size, bool isAtMode)
{
	HAL_StatusTypeDef transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, dataToSend, size, UART_TRANSMITION_TIMEOUT_MS);

	// Включаем прерывание по остановке приема данных
	HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(btHc05Uart.uartHandler, btHc05Uart.rxBuf, BT_HC_05_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(btHc05Uart.dmaUartRx, DMA_IT_HT);
}
uint8_t fullBuf[300];
uint32_t fullBufSize = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	static uint32_t entryCounter = 0;
	entryCounter++;
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	if(btHc05Uart.uartHandler != NULL && huart == btHc05Uart.uartHandler)
	{
		memcpy(fullBuf + fullBufSize,btHc05Uart.rxBuf, Size);
		fullBufSize += Size;
		if(strstr((char *)btHc05Uart.rxBuf, "OK"))
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		}
		LCD_Clear();
		LCD_SetPos(0, 0);
		char firstPartOfMessage[17] = { '\0' };

		memcpy(firstPartOfMessage,btHc05Uart.rxBuf, 16);
		LCD_String(firstPartOfMessage);

		if(Size > 16)
		{
			uint32_t notFittedBytes = Size - 16;
			if(notFittedBytes > 16)
			{
				notFittedBytes = 16;
			}

			char secondPartOfMessage[17] = { '\0' };
			memcpy(secondPartOfMessage,btHc05Uart.rxBuf + 16, notFittedBytes);

			LCD_SetPos(0, 1);
			LCD_String(secondPartOfMessage);
		}

//		btHc05Uart.rx_callback_function(btHc05Uart.rxBuf, Size);
		// ECHO
//		HAL_UART_Transmit(btHc05Uart.uartHandler, "AT+INQ?\r\n", 9, 100);
		// HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+INQ\r\n", 8, 3000);


		HAL_StatusTypeDef res = HAL_UARTEx_ReceiveToIdle_DMA(btHc05Uart.uartHandler, btHc05Uart.rxBuf, BT_HC_05_RX_BUF_SIZE);
				__HAL_DMA_DISABLE_IT(btHc05Uart.dmaUartRx, DMA_IT_HT);

	}
}

void bt_hc_05_init(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx)
{
	btHc05Uart.uartHandler = uartHandler;
	btHc05Uart.dmaUartRx = dmaUartRx;
	btHc05Uart.rxBuf = btBuffer;

	btHc05Uart.send_data_function = bt_hc_send_data;
	btHc05Uart.rx_callback_function = NULL;
}

///
void atResponseCallback_at_reset(uint8_t * responseData, uint32_t parameter)
{
	if(strstr(responseData, AT_COMMAND_OK_RESPONSE) != NULL)
	{

	}
}
void send_at_reset()
{
	btHc05Uart.rx_callback_function = atResponseCallback_at_reset;
	HAL_StatusTypeDef transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+RESET\r\n", 10, 3000);
}
///

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
		HAL_Delay(200);
		HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+RESET\r\n", 10, 3000);
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
	HAL_Delay(200);
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

enum SyncUartReceiveResult
{
	RECEIVE_OK,
	TIME_OUT,
	OVERFLOWED,
	FAILED
};
int32_t sync_receive_bytes_until_rn(uint8_t *rxBuf, uint32_t rxBufSize, uint32_t maxReceiveTimeMs)
{
	uint8_t * ptrToBuf = rxBuf;
	uint32_t sizeOfReceived = 0;

	uint32_t receiveBeginTimeStartMs = HAL_GetTick();

	while(HAL_GetTick() - receiveBeginTimeStartMs < maxReceiveTimeMs)
	{
		HAL_StatusTypeDef receiveResult = HAL_UART_Receive(btHc05Uart.uartHandler, ptrToBuf, 1, 100);

		if(receiveResult == HAL_OK)
		{
			if(*ptrToBuf == '\r')
			{
				return RECEIVE_OK;
			}

			sizeOfReceived++;
			ptrToBuf++;

			if(sizeOfReceived > rxBufSize)
			{
				return OVERFLOWED;
			}
		}
		else
		{
			return FAILED;
		}
	}
	return TIME_OUT;

}

// TODO: Функция запускает поиск дочернего устройства по имени
// Функцию может вызываться несколько раз уже в процессе поиска, ЭТО НЕ ДОЛЖНО ЛОМАТЬ ЛОГИКУ!
bool bt_hc_05_start_scan()
{
	uint8_t bufForSyncReplies[32] = { 0 };
	// Ищем до 10 устройств по rssi с таймаутом в 60 секунд.
	//AT+INQM=1,10,77\r\n


	HAL_StatusTypeDef transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+INQM=1,3,48\r\n", 16, 300);
	HAL_UART_Receive(btHc05Uart.uartHandler, bufForSyncReplies, 20, 4000);

	transRes = HAL_UARTEx_ReceiveToIdle_DMA(btHc05Uart.uartHandler, btHc05Uart.rxBuf, BT_HC_05_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(btHc05Uart.dmaUartRx, DMA_IT_HT);
	transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+INQ\r\n", 8, 300);

	return true;

}










