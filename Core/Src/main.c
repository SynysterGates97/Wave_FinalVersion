/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "stdbool.h"
#include "audioplay.h"
#include "bt_hc_05_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern ApplicationTypeDef Appli_state;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId defaultTaskHandle;
osThreadId ledTaskHandle;
osThreadId audioTaskHandle;
osThreadId menuTaskHandle;
/* USER CODE BEGIN PV */
extern volatile int menuState;
extern volatile int menuDown;
extern volatile int menuUp;
extern volatile int menuPlay;
extern volatile int menuIndex;
extern volatile int nFiles;

volatile int playing_changed;

extern uint8_t retUSBH;    /* Return value for USBH */
extern char USBHPath[4];   /* USBH logical drive path */
extern FATFS USBHFatFS;    /* File system object for USBH logical drive */
extern FIL USBHFile;       /* File object for USBH */

uint8_t usb_ok = 0;
bool test_readyToPlay = false;

FIL MyFile;                   /* File object */
FIL ConFile;
FIL WavFile;

char FilNam[20][50];

extern USBH_HandleTypeDef hUsbHostFS;


extern volatile AUDIO_StateMachine     Audio;
WAVE_FormatTypeDef *waveformat =  NULL;
uint32_t WaveDataLength = 0;
char str[100];
char FileName[100]={0};
uint8_t info[44];

uint8_t index_inplay = -1;

uint8_t playing_now=0;
uint8_t flag_opt =0;//flag showing that the current option now is playing.
uint8_t switch_opt = 0;//?????? ????? ????. ???.
uint8_t flag_stop_light = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_RNG_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartLedTask(void const * argument);
void StartAudioTask(void const * argument);
void StartMenuTask(void const * argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
static void GetFileList(void)
{
	FRESULT result; /* FatFs function common result code */
	/* File read buffer */

	DIR dir;
//   nFiles = 0;
	static FILINFO fileInfo;

	result = f_opendir(&dir, "/");

	if (result == FR_OK) //FR_NOT_ENABLED
	{
		result = f_readdir(&dir, &fileInfo); //??? ???? ????? ? ????? ???????? system volume
		while (((result = f_readdir(&dir, &fileInfo)) == FR_OK)
				&& fileInfo.fname[0])
		{
			if (strstr(fileInfo.fname, ".wav"))
			{
				uint32_t nameSizeInBytes = strlen(fileInfo.fname);
				strncpy(FilNam[nFiles], fileInfo.fname, nameSizeInBytes);

				nFiles++;
			}
		}
	}
	f_closedir(&dir);
}

void GetFileInfo(void)
{
   LCD_Clear();
   if(f_open(&WavFile,FilNam[menuIndex], FA_OPEN_EXISTING | FA_READ)==FR_OK)
   {
      uint32_t duration;
      uint32_t bytesread;
      if(f_read(&WavFile, info, 44, (void*)&bytesread)==FR_OK)
      {
         waveformat=(WAVE_FormatTypeDef*)info;
         sprintf((char*)str, "%d", (int)(waveformat->SampleRate));
         sprintf((char*)str, " %d", (int)(waveformat->NbrChannels));
         duration = waveformat->FileSize / waveformat->ByteRate;
         sprintf((char*)str, " %d %02d:%02d", (int)(waveformat->FileSize/1024),
                 (int)duration/60, (int)duration%60);
      }
      f_close(&WavFile);
   }
}
void MenuProcess(void)
{
	switch (Audio.state)
	{
		case AUDIO_IDLE:
			Audio.state = AUDIO_WAIT;
			break;
		case AUDIO_WAIT:
			LCD_SetPos(0, 0);
			f_close(&WavFile);
			GetFileInfo(); // TODO: BACK
			AudioPlay_Init(waveformat->SampleRate);
			WaveDataLength = waveformat->FileSize;
			Audio.state = AUDIO_PLAYBACK;
			break;
		case AUDIO_EXPLORE:
			break;
		case AUDIO_PLAYBACK:
			WaveDataLength = waveformat->FileSize;
			// TODO: FileName - нужно заменить на File
			if (f_open(&WavFile, FilNam[menuIndex], FA_OPEN_EXISTING | FA_READ) == FR_OK) {
				AudioPlay_Start(waveformat->SampleRate);
				f_close(&WavFile);
			}
			//LCD_Clear();
			sprintf(FileName, FilNam[menuIndex]);
			Audio.state = AUDIO_IDLE;
			menuPlay = 1; ///////////////////////////////////////////play is always 1
			// state = 1;
			break;
		case AUDIO_IN:
			break;
		default:
			break;
	}
}

#define RED    TIM3->CCR2
#define GREEN    TIM3->CCR3
#define BLUE    TIM3->CCR1

#define TOP    65535

volatile uint16_t BPM=120;
volatile uint16_t Nature=255;
volatile uint16_t Dynamic=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_RNG_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, StartLedTask, osPriorityBelowNormal, 0, 256);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of audioTask */
  osThreadDef(audioTask, StartAudioTask, osPriorityBelowNormal, 0, 512);
  audioTaskHandle = osThreadCreate(osThread(audioTask), NULL);

  /* definition and creation of menuTask */
  osThreadDef(menuTask, StartMenuTask, osPriorityBelowNormal, 0, 256);
  menuTaskHandle = osThreadCreate(osThread(menuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 102;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_HC_05_EN_GPIO_Port, BT_HC_05_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Down_Pin Up_Pin Play_Pin */
  GPIO_InitStruct.Pin = Down_Pin|Up_Pin|Play_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_HC_05_EN_Pin */
  GPIO_InitStruct.Pin = BT_HC_05_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_HC_05_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_HC_STATE_Pin */
  GPIO_InitStruct.Pin = BT_HC_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT_HC_STATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s3)
{
	if(hi2s3->Instance==I2S3)
	{
		AudioPlay_HalfTransfer_Callback();
	}
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s3)
{
	if(hi2s3->Instance==I2S3)
	{
		AudioPlay_Transfer_Callback();
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  uint32_t delayMs;

	  bool isBtInitialized = bt_state_machine_process_states(&delayMs, false);
	  if(!isBtInitialized)
	  {
		  bt_state_machine_start(&huart3, &hdma_usart3_rx);
	  }

	  osDelay(delayMs);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/

void fire_simulator_do_one_time_slot(uint16_t slotIndex)
{
	RED = slotIndex;
	GREEN = slotIndex*0.15;
	BLUE = slotIndex * 0;
}
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	uint32_t filterUpdTimeMs = 0;
	uint32_t writeFireTimeMs = 0;

	uint16_t writingValue = 0;
	uint16_t rndWritingValue = 0;

	float koefficient = 0.2;
  /* Infinite loop */
  for(;;)
  {
	  if(usb_ok)
	  {

		  if(HAL_GetTick() - filterUpdTimeMs > 400)
		  {
			  rndWritingValue = HAL_RNG_GetRandomNumber(&hrng);
			  filterUpdTimeMs = HAL_GetTick();
		  }

		  if(HAL_GetTick() - writeFireTimeMs > 100)
		  {
			  writingValue = writingValue * (1 - koefficient) + (float)rndWritingValue * koefficient;
			  writeFireTimeMs = HAL_GetTick();

			  fire_simulator_do_one_time_slot(writingValue);


		  }

//		  uint16_t random = HAL_RNG_GetRandomNumber(&hrng);
//		  fire_simulator_do_one_time_slot(random);

//		  osDelay(1);
//		  option1();
	  }
	  osDelay(1);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartAudioTask */
/**
* @brief Function implementing the audioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioTask */
void StartAudioTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioTask */
	HAL_TIM_Base_Start(&htim2);

	LCD_ini();
	LCD_Clear();

	LCD_SetPos(0, 0);
	LCD_String("Mark V");
	HAL_Delay(2000);
  /* Infinite loop */
  for(;;)
  {

	  if(playing_now)
	  {
		 MenuProcess();
	  }

//		char str[17];
//		sprintf(str, "u:%d|d:%d|i=%d|s:%d", menuUp, menuDown, menuIndex,
//				menuState);
//
//		LCD_Clear();
//		LCD_SetPos(0, 0);
//		LCD_String(str);

		osDelay(1);
  }
  /* USER CODE END StartAudioTask */
}

/* USER CODE BEGIN Header_StartMenuTask */
/**
* @brief Function implementing the menuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMenuTask */
void StartMenuTask(void const * argument)
{
  /* USER CODE BEGIN StartMenuTask */
  /* Infinite loop */
	for (;;)
	{
		if (Appli_state == APPLICATION_READY && !usb_ok)
		{
			GetFileList();
			usb_ok = TRUE;
			menuState = 1;
			menuDown = 1;
		}
		else if (usb_ok)
		{
			if (playing_changed)
			{
				// GetConfig(); // Раньше так считывались конфигурационные файлы, в которых была информация о свет. сопровождении.
				switch_opt ^= 1;
				flag_stop_light = 1;
				playing_changed = 0;
			}
			if (menuState)
			{
				if (menuDown)
				{
					if (menuIndex % 2 == 0)
					{
						LCD_Clear();
						LCD_SetPos(0, 0);
						LCD_String(">");
						LCD_SetPos(1, 0);
						LCD_String(FilNam[menuIndex]);
						LCD_SetPos(0, 1);
						LCD_String(" ");
						LCD_SetPos(1, 1);
						LCD_String(FilNam[menuIndex + 1]);
					}
					else
					{
						LCD_Clear();
						LCD_SetPos(0, 0);
						LCD_String(" ");
						LCD_SetPos(1, 0);
						LCD_String(FilNam[menuIndex - 1]);
						LCD_SetPos(0, 1);
						LCD_String(">");
						LCD_SetPos(1, 1);
						LCD_String(FilNam[menuIndex]);
					}
					menuDown = 0;
				}
				if (menuUp)
				{
					if (menuIndex % 2 == 0)
					{
						LCD_Clear();
						LCD_SetPos(0, 0);
						LCD_String(">");
						LCD_SetPos(1, 0);
						LCD_String(FilNam[menuIndex]);
						LCD_SetPos(0, 1);
						LCD_String(" ");
						LCD_SetPos(1, 1);
						LCD_String(FilNam[menuIndex + 1]);
					}
					else
					{
						LCD_Clear();
						LCD_SetPos(0, 0);
						LCD_String(" ");
						LCD_SetPos(1, 0);
						LCD_String(FilNam[menuIndex - 1]);
						LCD_SetPos(0, 1);
						LCD_String(">");
						LCD_SetPos(1, 1);
						LCD_String(FilNam[menuIndex]);
					}
					menuUp = 0;
				}
				if (menuPlay == 1)
				{
					sprintf(FileName, FilNam[menuIndex]);

					if (playing_now == 0)
						playing_changed = 1;
					playing_now = 1;

				}
				menuState = 0;
			}
		}
		osDelay(1);
	}
  /* USER CODE END StartMenuTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

