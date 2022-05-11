#include "bt_hc_05_driver.h"
#include "stm32f407xx.h"
#include "Necomimi/necomimi.h"
#include "Necomimi/necomimi_queue.h"

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

//"AlphaWaveSon"
#define WAVE_CHILD_DEVICE_NAME "VovaTheDevil"

#define AT_COMMAND_OK_RESPONSE "OK"

///////////////////////////
#define UART_TRANSMITION_TIMEOUT_MS 1000

bool isAtModeEnabled = false;

uint8_t btBuffer[BT_HC_05_RX_BUF_SIZE] = { 0 };


// 705E,55,BF0F46
// +INQ:705E:55:BF0F46,5A020C,FFC9,realme 3\r\n
#define BT_SON_ADRESS_FOR_AT_BIND_STRING_SIZE 15

#define AT_INQ_RESPONSE_HEADER				"+INQ:"
#define AT_INQ_RESPONSE_HEADER_SIZE 		5
#define AT_INQ_RESPONSE_BT_MAC_ADDR_SIZE 	14

#define AT_INQ_RESPONSE_BT_ADDR_BEGIN_OFFSET 	AT_INQ_RESPONSE_HEADER_SIZE
#define AT_INQ_RESPONSE_BT_ADDR_FIRST_COLON_OFFSET 	AT_INQ_RESPONSE_HEADER_SIZE + 4
#define AT_INQ_RESPONSE_BT_ADDR_SECOND_COLON_OFFSET 	AT_INQ_RESPONSE_HEADER_SIZE + 7

#define BT_MAC_ADDR_NAP_STRING_SIZE 4
#define BT_MAC_ADDR_UAP_STRING_SIZE 2
#define BT_MAC_ADDR_LAP_STRING_SIZE 6



volatile char btSonAddressStringForAtBind[BT_SON_ADRESS_FOR_AT_BIND_STRING_SIZE] = { 0 };
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

static bool parse_son_address_in_at_inq_response(const uint8_t *hc05AnswerData, uint32_t answerSize)
{
	// 705E,55,BF0F46 - хранить адрес будем в таком формате для удобства HC_05
	// +INQ:705E:55:BF0F46,5A020C,FFC9,realme 3\r\n - Приходит в таком формате
	//	#define BT_SON_ADRESS_FOR_AT_BIND_STRING_SIZE 15
	//	char btSonAddressStringForAtBind[BT_SON_ADRESS_FOR_AT_BIND_STRING_SIZE] = { 0 };

	uint8_t *ptrToAnswerData = hc05AnswerData;

	if(answerSize > AT_INQ_RESPONSE_HEADER_SIZE + AT_INQ_RESPONSE_BT_MAC_ADDR_SIZE )
	{
		ptrToAnswerData = strstr(ptrToAnswerData, AT_INQ_RESPONSE_HEADER);
		uint32_t bytesLeft = answerSize - (ptrToAnswerData - hc05AnswerData);
		if(ptrToAnswerData && bytesLeft > AT_INQ_RESPONSE_BT_MAC_ADDR_SIZE)
		{
			if(ptrToAnswerData[AT_INQ_RESPONSE_BT_ADDR_FIRST_COLON_OFFSET] == ':' &&
					ptrToAnswerData[AT_INQ_RESPONSE_BT_ADDR_SECOND_COLON_OFFSET] == ':')
			{
				char *ptrToCopyingAddr = btSonAddressStringForAtBind;
				memcpy((uint8_t*)ptrToCopyingAddr, ptrToAnswerData + AT_INQ_RESPONSE_HEADER_SIZE, BT_MAC_ADDR_NAP_STRING_SIZE);
				ptrToCopyingAddr += BT_MAC_ADDR_NAP_STRING_SIZE;
				*(ptrToCopyingAddr++) = ',';

				memcpy(ptrToCopyingAddr,
						ptrToAnswerData + AT_INQ_RESPONSE_BT_ADDR_FIRST_COLON_OFFSET + 1,
						BT_MAC_ADDR_UAP_STRING_SIZE);
				ptrToCopyingAddr += BT_MAC_ADDR_UAP_STRING_SIZE;
				*(ptrToCopyingAddr++) = ',';

				memcpy(ptrToCopyingAddr,
										ptrToAnswerData + AT_INQ_RESPONSE_BT_ADDR_SECOND_COLON_OFFSET + 1,
										BT_MAC_ADDR_LAP_STRING_SIZE);

				// Ноль символ в btSonAddressStringForAtBind можно не заполнять, он уже там.
				return true;
			}

		}
	}
	return false;
}


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
		bool needToActivateDma = true;
		bool sonFound = btSonAddressStringForAtBind[0] != '\0';
		if (sonFound)
		{
			necomimi_parse_packet(btHc05Uart.rxBuf, Size);
		}
		else if (strstr(btHc05Uart.rxBuf, WAVE_CHILD_DEVICE_NAME))
		{
			sonFound = parse_son_address_in_at_inq_response(btHc05Uart.rxBuf, Size);
			needToActivateDma = !sonFound;
		}

		if(needToActivateDma)
		{
			HAL_StatusTypeDef res = HAL_UARTEx_ReceiveToIdle_DMA(btHc05Uart.uartHandler, btHc05Uart.rxBuf, BT_HC_05_RX_BUF_SIZE);
			__HAL_DMA_DISABLE_IT(btHc05Uart.dmaUartRx, DMA_IT_HT);
		}

		memset(btHc05Uart.rxBuf, 0, BT_HC_05_RX_BUF_SIZE);
	}
}

void bt_hc_05_init(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx)
{
	btHc05Uart.uartHandler = uartHandler;
	btHc05Uart.dmaUartRx = dmaUartRx;
	btHc05Uart.rxBuf = btBuffer;

	btHc05Uart.send_data_function = bt_hc_send_data;
	btHc05Uart.rx_callback_function = NULL;

	necomimi_queue_init();
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
		HAL_Delay(400);
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
}

void bt_hc_05_bind_to_father()
{
	HAL_StatusTypeDef transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+INQC\r\n", strlen("AT+INQC\r\n"), 1000);

	static char atBindCommand[50] = { 0 };
	static char atLinkCommand[50] = { 0 };
	sprintf(atBindCommand, "AT+BIND=%s\r\n", btSonAddressStringForAtBind);
	sprintf(atLinkCommand, "AT+LINK=%s\r\n", btSonAddressStringForAtBind);

	uint32_t commandLen = strlen(atBindCommand);

	transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+CMODE=0\r\n", 12, 1000);
	transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)atBindCommand, commandLen, 1000);
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

	HAL_StatusTypeDef transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+ROLE=1\r\n", strlen("AT+ROLE=1\r\n"), 300);
	HAL_UART_Receive(btHc05Uart.uartHandler, bufForSyncReplies, 20, 200);

	transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+RMAAD\r\n", strlen("AT+RMAAD\r\n"), 300);
	HAL_UART_Receive(btHc05Uart.uartHandler, bufForSyncReplies, 20, 200);

	transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+INQM=1,8,48\r\n", 16, 300);
	HAL_UART_Receive(btHc05Uart.uartHandler, bufForSyncReplies, 20, 4000);

	transRes = HAL_UARTEx_ReceiveToIdle_DMA(btHc05Uart.uartHandler, btHc05Uart.rxBuf, BT_HC_05_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(btHc05Uart.dmaUartRx, DMA_IT_HT);
	transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)"AT+INQ\r\n", 8, 300);

	return true;

}

void bt_hc_05_activate_read()
{
	HAL_StatusTypeDef transRes = HAL_UARTEx_ReceiveToIdle_DMA(btHc05Uart.uartHandler, btHc05Uart.rxBuf, BT_HC_05_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(btHc05Uart.dmaUartRx, DMA_IT_HT);
}

void bt_hc_05_send_string(char *message)
{
	HAL_StatusTypeDef transRes = HAL_UART_Transmit(btHc05Uart.uartHandler, (uint8_t*)message, strlen(message), 500);
}









