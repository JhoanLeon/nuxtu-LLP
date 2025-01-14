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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

// Define to activate debug mode with GPIO signal on code
//#define DEBUG_MODE

////// STATIC COMMANDS TO REQUEST DATA //////

#define REQUEST_DETECTION 0
#define REQUEST_DEVICE_TYPE 1
#define REQUEST_DEVICE_METADATA_BASIC 2
#define REQUEST_DEVICE_METADATA_COMPLETE 3
#define REQUEST_DEVICE_METADATA_VOLTAGE_DATA 4

////// STATIC INFORMATION ABOUT DEVICE //////

// REQUEST_DETECTION
uint8_t DETECTION_VALUE = 0x00;

// REQUEST_DEVICE_TYPE
uint8_t DEVICE_TYPE = 0x00;

// REQUEST_DEVICE_METADATA_BASIC
uint8_t BASIC_DATA[14] = {0x00, 0x0A, 3, '2','0','/','0','1','/','2','0','2','2', 0x05}; // BASIC_DATA[14] = {PCB_ID[15:8], PCB_ID[7:0], NUMBER_OF_SENSORS, MANUFACTURE_DATE, PCB_CAPABILITES};

// Defines for actual environmental sensors in the system
//#define TEMP
#define PCB_TEMP
//#define HUMD
#define PRES

// REQUEST_DEVICE_METADATA_COMPLETE
#define gas_sensors 3 // at least one gas sensor in the array

float n1_voltage = 0.0; // mV for n1 gas sensor
float n2_voltage = 0.0; // mV for n2 gas sensor
float n3_voltage = 0.0; // mV for n3 gas sensor

typedef struct gas_sensor
{
	unsigned char name[11];
	unsigned char type[14];
	unsigned char main_gas[20];
	uint16_t response_time;
} gas_sensor;

uint8_t COMPLETE_DATA[47*gas_sensors];
uint8_t COMPLETE_DATA1[47];
uint8_t COMPLETE_DATA2[47];
uint8_t COMPLETE_DATA3[47];

// REQUEST_DEVICE_METADATA_VOLTAGE_DATA
#define env_sensors 2 // environmental sensors

float VSENSE = 3.3/4095; // constant to conversions of ADC value to V
float V25 = 0.76; // Voltage of internal temperature sensor at 25°C
float Avg_slope = 0.0025; // V/degC

float pcb_temperature = 0.0; // initialized to return °C of PCB temperature
float temperature = 0.0; // initialized to return °C of environment temperature
float humidity = 0.0; // initialized to return %RH of environment humidity
float pressure = 0.0; // initialized to return kPa of absolute pressure

// OTHER VARIABLES
uint8_t last_command_received = 0;
uint8_t ERROR_CALL = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// function to check address matching
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if ( TransferDirection == I2C_DIRECTION_TRANSMIT ) // Master sends a write instruction
	{
		// read byte sent by master (this is the request information command)
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, &last_command_received, sizeof(last_command_received), I2C_NEXT_FRAME);
	}

	else // Master sends a read instruction
	{
		switch(last_command_received)
		{
			case REQUEST_DETECTION:
			{
				// the master did not request a value, it is just for detection (offset = 0)
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &DETECTION_VALUE, sizeof(DETECTION_VALUE), I2C_NEXT_FRAME);
				break;
			}

			case REQUEST_DEVICE_TYPE:
			{
				// SEND DATA
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &DEVICE_TYPE, sizeof(DEVICE_TYPE), I2C_NEXT_FRAME);
				break;
			}

			case REQUEST_DEVICE_METADATA_BASIC:
			{
				// SEND DATA
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, BASIC_DATA, sizeof(BASIC_DATA), I2C_NEXT_FRAME);
				break;
			}

			case REQUEST_DEVICE_METADATA_COMPLETE:
			{
				// SEND DATA FOR EACH GAS SENSOR
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, COMPLETE_DATA, sizeof(COMPLETE_DATA), I2C_NEXT_FRAME);
				break;
			}

			case REQUEST_DEVICE_METADATA_VOLTAGE_DATA:
			{
				// SEND ENVIRONMENTAL VARIABLES DATA
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)&temperature, 16+4*gas_sensors, I2C_NEXT_FRAME); // treat float values as a buffer to send through I2C
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)&pcb_temperature, 16+4*gas_sensors, I2C_NEXT_FRAME); // treat float values as a buffer to send through I2C
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)&humidity, 16+4*gas_sensors, I2C_NEXT_FRAME); // treat float values as a buffer to send through I2C
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)&pressure, 16+4*gas_sensors, I2C_NEXT_FRAME); // treat float values as a buffer to send through I2C

				// SEND GAS SENSORS VOLTAGES
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)&n1_voltage, 16+4*gas_sensors, I2C_NEXT_FRAME); // treat float values as a buffer to send through I2C
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)&n2_voltage, 16+4*gas_sensors, I2C_NEXT_FRAME); // treat float values as a buffer to send through I2C
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)&n3_voltage, 16+4*gas_sensors, I2C_NEXT_FRAME); // treat float values as a buffer to send through I2C
				break;
			}

			default:
			{
				break;
			}
		}
		last_command_received = 0; // to repeat detection
	}
}

// function to put the slave ready again when listen mode finish
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

#ifdef DEBUG_MODE
// function to detect complete reception of information
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14); // just for quick debug
}

// function to detect complete transmission of information
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14); // just for quick debug
}

// function to manage i2c errors
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF ) // Error obtained in debug, due to no clock stretching
	{
		ERROR_CALL = 1;
	}
	if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_OVR ) // Error obtained in debug, due to no clock stretching
	{
		ERROR_CALL = 2;
	}
	if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_TIMEOUT ) // Error obtained in debug, due to no clock stretching
	{
		ERROR_CALL = 3;
	}
}
#endif

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_I2C_EnableListen_IT(&hi2c1); // to activate the slave mode of I2C

  #ifdef DEBUG_MODE
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, RESET);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, RESET);
  #endif

  uint8_t total_adc_sensors = env_sensors + gas_sensors; // total ADC channels to be used for analog sensors

  uint32_t adc_values[total_adc_sensors]; // to store all read ADC values, in channel name order (channel1 -> adc[0], ..., temp -> adc[total-1])

  //PA1 -> pressure
  //PA2 -> FEC
  //PA3 -> CO
  //PF4 -> 4-NOC

  HAL_ADC_Start_DMA(&hadc1, adc_values, sizeof(adc_values)); // start the ADC in DMA mode

  gas_sensor N1 = {"FECS40-1000", "Electroquimico", "Monoxido de carbono", 30};
  gas_sensor N2 = {"CO-CE-10000", "Electroquimico", "Monoxido de carbono", 75};
  gas_sensor N3 = {"4-NO2-20000", "Electroquimico", "Dioxido de nitrogeno", 60};
  gas_sensor N4 = {"X-XXX-XXXXX", "XXXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXX", 0}; // dummy value

  int i = 0;
  for (int k = 0; k < 11; k++)
  {
	  COMPLETE_DATA1[i] = N1.name[k];
	  i++;
  }
  for (int k = 0; k < 14; k++)
  {
	  COMPLETE_DATA1[i] = N1.type[k];
	  i++;
  }
  for (int k = 0; k < 20; k++)
  {
	  COMPLETE_DATA1[i] = N1.main_gas[k];
	  i++;
  }

  COMPLETE_DATA1[i] =  (N1.response_time >> 8) & 0xFF;
  i++;
  COMPLETE_DATA1[i] = (N1.response_time) & 0xFF;

  i = 0;
  for (int k = 0; k < 11; k++)
  {
	  COMPLETE_DATA2[i] = N2.name[k];
	  i++;
  }
  for (int k = 0; k < 14; k++)
  {
	  COMPLETE_DATA2[i] = N2.type[k];
	  i++;
  }
  for (int k = 0; k < 20; k++)
  {
	  COMPLETE_DATA2[i] = N2.main_gas[k];
	  i++;
  }


  COMPLETE_DATA2[i] = (N2.response_time >> 8) & 0xFF;
  i++;
  COMPLETE_DATA2[i] = (N2.response_time) & 0xFF;

  i = 0;
  for (int k = 0; k < 11; k++)
  {
	  COMPLETE_DATA3[i] = N3.name[k];
	  i++;
  }
  for (int k = 0; k < 14; k++)
  {
	  COMPLETE_DATA3[i] = N3.type[k];
	  i++;
  }
  for (int k = 0; k < 20; k++)
  {
	  COMPLETE_DATA3[i] = N3.main_gas[k];
	  i++;
  }

  COMPLETE_DATA3[i] =  (N3.response_time >> 8) & 0xFF;
  i++;
  COMPLETE_DATA3[i] =  (N3.response_time) & 0xFF;

  int j = 0;
  for (int i = 0; i < 47; i++)
  {
	  COMPLETE_DATA[j] = COMPLETE_DATA1[i];
	  j++;
  }
  for (int i = 0; i < 47; i++)
  {
	  COMPLETE_DATA[j] = COMPLETE_DATA2[i];
	  j++;
  }
  for (int i = 0; i < 47; i++)
  {
	  COMPLETE_DATA[j] = COMPLETE_DATA3[i];
	  j++;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ////// COMPUTATION OF ENVIRONMENTAL VALUES //////

	  #ifdef TEMP
	  temperature = -66.875 + 218.75*(adc_values[0]/4095); // formula taken from datasheet of SHT31-ARP-B pag.8 (°C -45 -> +125)
	  #endif

	  #ifdef HUMD
	  humidity = -12.5 + 125*(adc_values[1]/4095); // formula taken from datasheet of SHT31-ARP-B pag.8 (%RH 0 -> 100)
	  #endif

	  #ifdef PRES
	  pressure = ((adc_values[0]/4095) - 0.05069)/0.00293; // formula derived from datasheet of KP229E2701 pag.12
	  #endif

	  #ifdef PCB_TEMP
	  pcb_temperature = ((V25 - adc_values[total_adc_sensors-1]*VSENSE)/ Avg_slope) + 25; // formula taken from reference manual of STM32F303VCT6 pag.373
	  #endif

	  ////// COMPUTATION OF GAS SENSORS VOLTAGES //////

	  n1_voltage = adc_values[1]*VSENSE*1000; // mV from n_1 gas sensor
	  n2_voltage = adc_values[2]*VSENSE*1000; // mV from n_2 gas sensor
	  n3_voltage = adc_values[3]*VSENSE*1000; // mV from n_3 gas sensor

	  #ifdef DEBUG_MODE
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8); // blink led to know that main program is running
	  #endif

	  HAL_Delay(50); // dummy delay, can be changed to a strategy to compute ADC values with callback

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 26;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
