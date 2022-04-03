#ifndef BT_HC_05_DRIVER_H_
#define BT_HC_05_DRIVER_H_

#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "bt_hc_05_driver.h"

extern volatile char btSonAddressStringForAtBind[];

void bt_hc_05_init(UART_HandleTypeDef *uartHandler, DMA_HandleTypeDef *dmaUartRx);
bool bt_state_machine_process_states(uint32_t *delayBeforeNextUpdateMs, bool needToWaitForConfiguration);

void bt_hc_05_switch_device_mode(bool isGoToAtMode);

bool bt_hc_05_connect_to_child_device();
bool bt_hc_05_find_child_device();
void bt_hc_05_bind_to_father();

bool bt_hc_05_start_scan();
void bt_hc_05_activate_read();
void bt_hc_05_send_string(char *message);
#endif /* BT_HC_05_DRIVER_H_ */
