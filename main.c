/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM_MATH_CM4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId soundGenerationHandle;
osThreadId viewProcessingHandle;
osThreadId blueButtonPolliHandle;
osMutexId oobMutexHandle;
/* USER CODE BEGIN PV */
int xPosition=50;
int yPosition=50;
char buffer[100]; //for printing

float volume = 0.5;

int C6[42];
int D6[37];
int E6[33];
int F6[31];
int G6[28];
int A6[25];
int B6[22];

int output[50];

int lastTone = -1;
int toneCursor = 50;
int isPressed =0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartSoundGeneration(void const * argument);
void StartViewProcessing(void const * argument);
void StartBlueButtonPolling(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void readJoystick() {
	//X-Axis
	setADCChannel(1);
	HAL_ADC_Start(&hadc1);
	xPosition = ((float) HAL_ADC_GetValue(&hadc1) / 255) *100; //value between 0 and 100
	xPosition = xPosition > 43 && xPosition < 57 ? 50 : xPosition;
	HAL_ADC_Stop(&hadc1);

	//Y-Axis
	setADCChannel(2);
	HAL_ADC_Start(&hadc1);
	yPosition = ((float) HAL_ADC_GetValue(&hadc1) / 255) *100; //value between 0 and 100
	yPosition = yPosition > 43 && yPosition < 57 ? 50 : yPosition;
	HAL_ADC_Stop(&hadc1);
}

void setADCChannel(int channel) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = channel == 1 ? ADC_CHANNEL_1 : ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void setToneCursor() {
	osMutexWait(oobMutexHandle, osWaitForever);
	toneCursor += ((float) yPosition - 50) / 50 * 4;
	if (toneCursor > 100) toneCursor = 100;
	if (toneCursor < 0) toneCursor = 0;
	osMutexRelease(oobMutexHandle);
}

void setVolume() {
	osMutexWait(oobMutexHandle, osWaitForever);
	volume += ((float) xPosition - 50) /100 / 50 * 4 ;
	if (volume > 1) volume = 1;
	if (volume < 0) volume = 0;
	osMutexRelease(oobMutexHandle);
}

void setOutputSound(int* tone, int toneSize, int* destination, float volume) {
	for(int i=0; i<toneSize; i++) {
		destination[i] = (int) (tone[i] *volume);
	}
}

void setTone() {
	if (!isPressed) {
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		lastTone = -1;
		return;
	}
	if(toneCursor >= 0 && toneCursor < 14 && lastTone != 0) {
		lastTone =0;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		setOutputSound(C6, 42, output, volume);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, output, 42, DAC_ALIGN_8B_R);

	} else if (toneCursor >= 14 && toneCursor < 29 && lastTone != 1) {
		lastTone =1;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		setOutputSound(D6, 37, output, volume);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, output, 37, DAC_ALIGN_8B_R);

	} else if (toneCursor >= 29 && toneCursor < 43 && lastTone != 2) {
		lastTone =2;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		setOutputSound(E6, 33, output, volume);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, output, 33, DAC_ALIGN_8B_R);

	} else if (toneCursor >= 43 && toneCursor < 57 && lastTone != 3) {
		lastTone =3;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		setOutputSound(F6, 31, output, volume);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, output, 31, DAC_ALIGN_8B_R);

	} else if (toneCursor >= 57 && toneCursor < 71 && lastTone != 4) {
		lastTone =4;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		setOutputSound(G6, 28, output, volume);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, output, 28, DAC_ALIGN_8B_R);

	} else if (toneCursor >= 71 && toneCursor < 85 && lastTone != 5) {
		lastTone =5;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		setOutputSound(A6, 25, output, volume);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, output, 25, DAC_ALIGN_8B_R);

	} else if (toneCursor >= 85 && toneCursor <= 100 && lastTone != 6) {
		lastTone =6;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		setOutputSound(B6, 22, output, volume);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, output, 22, DAC_ALIGN_8B_R);
	}
}

/**
 * This function will print all the values that are of interest for demo and debugging (accelero, position, pitch, ...)
 */
void printValues() {
	//Printing joystick values
	memset(buffer, 0, 100);

	sprintf(buffer, "Joystick:\r\n\tx = %d\r\n\ty = %d\r\n\r\n", xPosition,yPosition);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	//Printing pitch and volume
	memset(buffer, 0, 100);

	sprintf(buffer, "ToneCursor = %d\tVolume = %.2f\r\n\r\n\r\n",
			toneCursor,
			volume);

	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

}

void fillTones(){
	fillToneArray(C6,42);
	fillToneArray(D6,37);
	fillToneArray(E6,33);
	fillToneArray(F6,31);
	fillToneArray(G6,28);
	fillToneArray(A6,25);
	fillToneArray(B6,22);
}

void fillToneArray(int* array, int arraySize) {
	for (int i=0; i<arraySize; i++) {
		array[i] = (int) (255*(0.5* arm_sin_f32(2* PI * ((float) i)/arraySize)+0.5));
	}
}

void printGUI() {
	memset(buffer, 0, 100);
	sprintf(buffer, "-----------------------------------------------------------------------------\r\n\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, " _______    ______     _______    _______    _______    _______    ______    \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, "(  ____ \\  (  __  \\   (  ____ \\  (  ____ \\  (  ____ \\  (  ___  )  (  ___ \\   \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, "| (    \\/  | (  \\  )  | (    \\/  | (    \\/  | (    \\/  | (   ) |  | (   ) )  \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, "| |        | |   ) |  | (__      | (__      | |        | (___) |  | (__/ /   \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, "| |        | |   | |  |  __)     |  __)     | | ____   |  ___  |  |  __ (    \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, "| |        | |   ) |  | (        | (        | | \\_  )  | (   ) |  | (  \\ \\   \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, "| (____/\\  | (__/  )  | (____/\\  | )        | (___) |  | )   ( |  | )___) )  \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, "(_______/  (______/   (_______/  |/         (_______)  |/     \\|  |/ \\___/   \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	osMutexWait(oobMutexHandle, osWaitForever);
	int markerPosition = (float)toneCursor/100 *77;
	osMutexRelease(oobMutexHandle);

	memset(buffer, 0, 100);
	for (int i=0; i<markerPosition-1; i++) {
		buffer[i] = ' ';
	}
	buffer[markerPosition] = '^';
	buffer[markerPosition+1] ='\r';
	buffer[markerPosition+2] ='\n';
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	for (int i=0; i<markerPosition-1; i++) {
		buffer[i] = ' ';
	}
	buffer[markerPosition] = '|';
	buffer[markerPosition+1] ='\r';
	buffer[markerPosition+2] ='\n';
	buffer[markerPosition+3] ='\r';
	buffer[markerPosition+4] ='\n';
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	osMutexWait(oobMutexHandle, osWaitForever);
	float volumePosition = volume;
	osMutexRelease(oobMutexHandle);

	memset(buffer, 0, 100);
	sprintf(buffer,"\t%c\r\n", volumePosition >=0.80 ? 'o' : '|');
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer,"\t%c\r\n", volumePosition >=0.60 && volumePosition <0.80 ? 'o' : '|');
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer,"Volume\t%c\r\n", volumePosition >=0.40 && volumePosition <0.60 ? 'o' : '|');
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer,"\t%c\r\n", volumePosition >=0.20 && volumePosition <0.40 ? 'o' : '|');
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer,"\t%c\r\n", volumePosition >=0.01 && volumePosition <0.20 ? 'o' : '|');
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

	memset(buffer, 0, 100);
	sprintf(buffer,"\t%c\r\n\r\n", volumePosition <0.01 ? 'o' : '|');
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 1000);

}

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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  fillTones();
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of oobMutex */
  osMutexDef(oobMutex);
  oobMutexHandle = osMutexCreate(osMutex(oobMutex));

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
  /* definition and creation of soundGeneration */
  osThreadDef(soundGeneration, StartSoundGeneration, osPriorityNormal, 0, 128);
  soundGenerationHandle = osThreadCreate(osThread(soundGeneration), NULL);

  /* definition and creation of viewProcessing */
  osThreadDef(viewProcessing, StartViewProcessing, osPriorityIdle, 0, 128);
  viewProcessingHandle = osThreadCreate(osThread(viewProcessing), NULL);

  /* definition and creation of blueButtonPolli */
  osThreadDef(blueButtonPolli, StartBlueButtonPolling, osPriorityIdle, 0, 128);
  blueButtonPolliHandle = osThreadCreate(osThread(blueButtonPolli), NULL);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2727;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blueButton_Pin */
  GPIO_InitStruct.Pin = blueButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(blueButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSoundGeneration */
/**
  * @brief  Function implementing the soundGeneration thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSoundGeneration */
void StartSoundGeneration(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(50);
    readJoystick();
    setToneCursor();
    setVolume();
    setTone();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartViewProcessing */
/**
* @brief Function implementing the viewProcessing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartViewProcessing */
void StartViewProcessing(void const * argument)
{
  /* USER CODE BEGIN StartViewProcessing */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    printGUI();
  }
  /* USER CODE END StartViewProcessing */
}

/* USER CODE BEGIN Header_StartBlueButtonPolling */
/**
* @brief Function implementing the blueButtonPolli thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlueButtonPolling */
void StartBlueButtonPolling(void const * argument)
{
  /* USER CODE BEGIN StartBlueButtonPolling */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);
	  if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
		isPressed =1;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		while (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {}
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		isPressed =0;
	  }
  }
  /* USER CODE END StartBlueButtonPolling */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
