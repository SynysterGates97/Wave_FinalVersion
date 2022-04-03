#include "bt_state_machine.h"
#include "bt_hc_05_driver.h"
uint32_t btState;

#define DEFAULT_DELAY_BETWEEN_UPDATES_MS 200
#define MAXIMUM_SCAN_TIME_MS (1 * 60 * 1000)

uint32_t scanStartTimeMs = 0;


// Флаг needToWaitForConfiguration показывает то, что использутся подключение к BT-конфигуратору
bool bt_state_machine_process_states(uint32_t *delayBeforeNextUpdateMs, bool needToWaitForConfiguration)
{
	*delayBeforeNextUpdateMs = DEFAULT_DELAY_BETWEEN_UPDATES_MS;

	bool isMachineInitialized = btState != BT_STATE_NOT_INITIALIZED;

	if(!isMachineInitialized)
	{
		return false;
	}

	switch (btState)
	{
		case BT_STATE_INITIALIZING:
			{
				LCD_Clear();
				LCD_SetPos(0, 0);
				LCD_String("BT_STATE_INITIALIZING");

				*delayBeforeNextUpdateMs = 1000;
				// Здесь в будущем возможно будет условный переход в другие состояния,
				// но сейчас безусловно переходим в состояние активации режима AT-команд.

				// ! На данный момент структура btHc05Uart уже готова к работе.
				btState = BT_STATE_AT_MODE_ENABLING;
			}
		break;

		case BT_STATE_AT_MODE_ENABLING:
			{
				LCD_Clear();
				LCD_SetPos(0, 0);
				LCD_String("BT_STATE_AT_M");

				bt_hc_05_switch_device_mode(true);
				btState = BT_STATE_START_SCAN;
				*delayBeforeNextUpdateMs = 300;
			}
		break;

		case BT_STATE_START_SCAN:
			{
				LCD_Clear();
				LCD_SetPos(0, 0);
				LCD_String("BT_STATE_START_SCAN");
				bt_hc_05_start_scan();

				scanStartTimeMs = HAL_GetTick();
				btState = BT_STATE_SCANNING;

			}
		break;

		case BT_STATE_SCANNING:
			{
				if(!needToWaitForConfiguration)
				{
					uint32_t totalScanTime = HAL_GetTick() - scanStartTimeMs;
					bool isScanTimeout = totalScanTime > MAXIMUM_SCAN_TIME_MS;

					char bufStr[20] = { 0 };

					sprintf(bufStr, "SCAN_TIME:%d", totalScanTime);

					LCD_Clear();
					LCD_SetPos(0, 0);
					LCD_String(bufStr);

					bool sonIsFound = btSonAddressStringForAtBind[0] != '\0';
					if(sonIsFound)
					{
						sonIsFound = true;
						LCD_Clear();
						LCD_SetPos(0, 0);
						LCD_String("FOUND THIS SHIT!");

						LCD_SetPos(0, 1);
						LCD_String(btSonAddressStringForAtBind);

						bt_hc_05_bind_to_father();
						btState = BT_STATE_DATA_MODE_ENABLING;


						// Только ради теста
						*delayBeforeNextUpdateMs = 3000;
						// goto BIND
					}
					else if(!isScanTimeout)
					{
						*delayBeforeNextUpdateMs = 500;
					}
					else
					{
						btState = BT_STATE_NOT_INITIALIZED;
						// TODO: лучше в таком случае делать полную перезагрузку, либо вывод на LCD хотя бы
					}
				}
				else
				{
					// TODO: здесь, если инициализировано подключение в режиме SLAVE
					// к устройству конфигуратору переходим в BT_STATE_CONFIG_MODE
				}
			}
		break;

		case BT_STATE_DATA_MODE_ENABLING:
			{
				bt_hc_05_switch_device_mode(false);

				btState = BT_STATE_ESTABLISHING_CONNECTION_WITH_CHILD;
			}
		break;

		case BT_STATE_ESTABLISHING_CONNECTION_WITH_CHILD:
			{
				bool isConnectionSuccess = bt_hc_05_connect_to_child_device();
				if(isConnectionSuccess == true)
				{
					btState = BT_STATE_PROCESSING_DATA_FROM_CHILD;
				}
			}
		break;

		case BT_STATE_PROCESSING_DATA_FROM_CHILD:
			{
				bt_hc_05_activate_read();
				*delayBeforeNextUpdateMs = 1000;
//				bt_hc_05_send_string("BYE!");
				// think_gear_process_data(currentDataInSomeFormat);
			}
		break;

		case BT_STATE_CONFIG_MODE:
			{

			}
		break;

		case BT_STATE_RESTART_MODULE:
			{

			}
		break;

		default:
			break;
	}
	return isMachineInitialized;
}

void bt_state_machine_start(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx)
{
	btState = BT_STATE_INITIALIZING;

	bt_hc_05_init(uartHandler,dmaUartRx);
}
