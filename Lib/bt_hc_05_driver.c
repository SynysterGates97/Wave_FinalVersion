///*
// * bt_hc_05_driver.c
// *
// *  Created on: Jan 28, 2022
// *      Author: Morgan
// */
//
//#include "bt_hc_05_driver.h"
//
//#include "stm32f4xx_hal_gpio.h"
//
//#define BT_HC_05_GPIO_STATE_PORT GPIOE
//#define BT_HC_05_GPIO_STATE_PIN GPIO_PIN_7
//
//#define BT_HC_05_GPIO_EN_PORT GPIOB
//#define BT_HC_05_GPIO_EN_PIN GPIO_PIN_1
//
//#define BT_HC_05_SET_EN_PIN() \
//	HAL_GPIO_WritePin(BT_HC_05_GPIO_STATE_PORT, BT_HC_05_GPIO_EN_PIN, GPIO_PIN_SET)
//
//#define BT_HC_05_RESET_EN_PIN() \
//	HAL_GPIO_WritePin(BT_HC_05_GPIO_STATE_PORT, BT_HC_05_GPIO_EN_PIN, GPIO_PIN_RESET)
//
//void bt_hc_05_switch_device_mode(bool isGoToAtMode)
//{
//	if (isGoToAtMode)
//	{
//		BT_HC_05_SET_EN_PIN();
//	}
//	else
//	{
//		BT_HC_05_RESET_EN_PIN();
//	}
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
