/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM32_Cap1203.h"
#include "ssd1306.h"
#include "ssd1306_conf_template.h"
#include "ssd1306_fonts.h"
#include "ble.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define filterwert 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint8_t cs_value[3] = {0,0,0};
uint8_t cs_oldvalue = 0;
int8_t page = 3;
uint8_t oled_off_merker = 1; //bedeutet Oled ist off
uint8_t newMessage = 0;

//osMessageQueueId_t data_msg;
uint8_t bleConnected = 0;
uint8_t EventFlag = 0;
uint8_t EventFlagOledOff = 0;
uint32_t oledOffCounter = 0;
uint8_t activindicator = 1;
uint32_t touchcounterCS1 = 0;
uint32_t touchcounterCS2 = 0;
uint32_t touchcounterCS3 = 0;
uint8_t scrolltimer = 0;

uint8_t textIndex = 0;

int8_t pagemerker = 0;
uint8_t powerOff = 0;

uint8_t wasPressed = 0;
//osThreadId_t id_data_sync;

int cs_state =0;

uint16_t touchDeBounce = 0;

uint8_t cs_val[filterwert] = {0};
char text[128] = {0};
char tempText[5] = {0};
//uint8_t max_sign = 6;
int cursor = 0;
char time[4] = {0};
char date[4] = {0};
uint32_t counter = 0;
uint32_t displayRefreshCounter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_IPCC_Init(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_RF_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void thread_oled_data(); 
void timeupdate();
void touchhelper();
void pageclickhelper();
void thread_touchdetection();
void thread_pageclick();
void thread_oled_auto_off();

	
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */
	
	//SystemClock_Config() macht ein Problem sobald RTC aktiviert ist HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK -> HAL_TIMEOUT
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
   MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_RF_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
  if(   (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
     && (__HAL_PWR_GET_FLAG(PWR_FLAG_C2SB) != RESET)
    )
  {
    // Clear Standby flag 
		
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB); 
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_C2SB);
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
  }
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	
	
	
	CAP1203_Init(hi2c1);
	//CAP1203_Int_Clr();
	//Falls Probleme, in Datei hw_timerserver.c Line 565 auskommentieren , Keine Ahung warum....
	CAP1203_Exit_PWR();
	CAP1203_Set_Standby_CS(ALL_CS_ENB);
	CAP1203_Set_Standby_Sensitivity(SENSITIVITY_8X);
	CAP1203_Go_Standby(CYCLETIME_70ms); 
	//CAP1203_Set_Sensitivity(SENSITIVITY_32X);
	
	
//	 HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_SET);
//	 osDelay(5);
//	ssd1306_Init();
//	ssd1306_SetContrast(50);
	//HAL_TIM_Base_Start_IT(&htim1); 
	TIM1->CCR3 = 0; 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	EventFlag = 2;
	
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
		
		thread_oled_data();
		thread_touchdetection();
		thread_pageclick();
		thread_oled_auto_off();
		
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OLED_RST_Pin|OLED_D_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OLED_CS_Pin|ADC_BAT_ENB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAP1203_INT_Pin */
  GPIO_InitStruct.Pin = CAP1203_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAP1203_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_RST_Pin OLED_D_C_Pin OLED_CS_Pin ADC_BAT_ENB_Pin */
  GPIO_InitStruct.Pin = OLED_RST_Pin|OLED_D_C_Pin|OLED_CS_Pin|ADC_BAT_ENB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_PWR_Pin */
  GPIO_InitStruct.Pin = OLED_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_PWR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void thread_touchdetection(){

	if(EventFlag == 1){
		/*int8_t page_merker = page;
		uint8_t cs_merker = 0;
		cs_value = CAP1203_Get_CS_State() & 7;
		
		for(int i = filterwert -1 ; i > 0; i--){
			cs_val[i] = cs_val[i-1];
		}
		cs_val[0] = cs_value;
		
		for(int i = 0; i <= filterwert; i++){
				cs_merker += cs_val[i];
		}
		if(cs_merker == (cs_value * filterwert)){ 
				EventFlag = 4;
		}
			
		if (cs_value != 0) cs_oldvalue = cs_value;
	
		cs_value = 0;
		//if(osTimerIsRunning(touchhelper) == 0) osTimerStart(touchhelper, 400);
		//if(osTimerIsRunning(pageclickhelper) == 0) osTimerStart(pageclickhelper, filterwert *250);
		
//		if(page >3)page = 0;
//		if(page <0)page = 3;
		
		if((page != page_merker) ||oled_off_merker) EventFlag = 2;
		
		HAL_Delay(200);
		CAP1203_Int_Clr();
		EventFlag = 2; */
		
			if(page == 3) {
				CAP1203_Int_Clr();
				cs_value[1] = CAP1203_Get_CS2_State();
			
				if(cs_value[1] == ON) {
					touchcounterCS2++;
					if(touchcounterCS2>3000) {
								page = 0;
								EventFlag = 4;
								touchcounterCS2 = 0; 
							}
				} else {
					touchcounterCS2 = 0;
				}
			}
			else {
		
				CAP1203_Int_Clr();
				
				cs_value[0] = CAP1203_Get_CS1_State();
				cs_value[1] = CAP1203_Get_CS2_State();
				cs_value[2] = CAP1203_Get_CS3_State();
				
				if(cs_value[0] == ON) {
					touchcounterCS1++;
				} else {
					touchcounterCS1 = 0;
				}
				
				if(cs_value[1] == ON) {
					touchcounterCS2++;
				} else {
					touchcounterCS2 = 0;
				}
				
				if(cs_value[2] == ON) {
					touchcounterCS3++;
				} else {
					touchcounterCS3 = 0;
				}
				
				if(wasPressed == 0) {
					
					if(oled_off_merker) {
						if(cs_value[0] == ON || cs_value[1] == ON || cs_value[2] == ON) {
							page = pagemerker;
							EventFlag = 2;
							wasPressed = 1;
						}
					}
					else { 
						
						if(cs_value[0] == ON || cs_value[1] == ON || cs_value[2] == ON) {
							wasPressed = 1;
						}
						
						if(cs_value[0] == ON) {
								/*page = 4;
								EventFlag = 2; */
						}
						else if(cs_value[1] == ON) {
								if(touchcounterCS2>3000) {
									page = 0;
									EventFlag = 4;
									touchcounterCS2 = 0; 
								}
						}
						else if(cs_value[2] == ON) {
							if(touchcounterCS3>1000) {
								page = 1;
								EventFlag = 2;
								touchcounterCS3 = 0; 
							}
						}
					}	
					touchDeBounce = 0;
				} 
				
				if(wasPressed == 1) {
					touchDeBounce++;
				}
				if(touchDeBounce >300) {
					wasPressed = 0;
				}
			}
			displayRefreshCounter++;
			
			if(displayRefreshCounter > 2000 && oled_off_merker != 1 && page != 2) { 	
				EventFlag = 2;
				displayRefreshCounter = 0;
			} 
			else if(displayRefreshCounter > 1250 && page == 2) {			//Microcontroller resettet wenn man eine höhere Refreshrate nimmt, keine Ahnung warum; max 1200
				EventFlag = 2;
				displayRefreshCounter = 0;
			} 
			
			if(powerOff == 2) {
				CAP1203_Exit_PWR();
				CAP1203_Set_Standby_CS(ALL_CS_ENB);
				CAP1203_Set_Standby_Sensitivity(SENSITIVITY_8X);
				CAP1203_Go_Standby(CYCLETIME_70ms);
				
				LL_PWR_ExitLowPowerRunMode();
				
				EventFlag = 2;
				page = 5;
				powerOff = 0;
			}
			
			/*cs_value[0] = 0;
			cs_value[1] = 0;
			cs_value[2] = 0;*/
	
	} 	
}

void thread_oled_data(){
		//osTimerStart(timeupdate, 60000); //Timer zum aktualisieren der Uhrzeit starten
	
	if(EventFlag == 2) {
		if(oled_off_merker){
			HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			ssd1306_Init();
			ssd1306_SetContrast(50);
			oled_off_merker = 0;
		}

		if(EventFlagOledOff != 8) {		//Oled Auto Off
			EventFlagOledOff = 8;
		}
		
		switch(page){
			case 1:
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();	
				ssd1306_SetCursor(11, 61);
				ssd1306_WriteString((char *)"SMS", Font_7x10, White);
				ssd1306_UpdateScreen();
				
				cursor = 14;					
			
				for(int i=0;i<5;i++) {
					tempText[i] = text[i];
				}
				
				if(newMessage) {
					TIM1->CCR3 = 50;
					HAL_Delay(200);
					TIM1->CCR3 = 0;
					HAL_Delay(200);
					TIM1->CCR3 = 50;
					HAL_Delay(200);
					TIM1->CCR3 = 0;
					HAL_Delay(400);
					
					
				}	else {
					HAL_Delay(1000);
				}
				newMessage = 0;
				page = 2;
			break;
			case 2:

				EventFlagOledOff = 0;
				oledOffCounter = 0;
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();
				ssd1306_SetCursor(cursor, 61);
				ssd1306_WriteChar(tempText[0], Font_7x10, White);
				ssd1306_WriteChar(tempText[1], Font_7x10, White);
				ssd1306_WriteChar(tempText[2], Font_7x10, White);
				ssd1306_WriteChar(tempText[3], Font_7x10, White);
				ssd1306_WriteChar(tempText[4], Font_7x10, White);
				ssd1306_UpdateScreen();
				
				cursor-=7;
				if(cursor<0) {
					if(tempText[0] == 0) {
						page = 0;
						counter = 0;
						cursor = 14;
					}
					else {
						counter++;
						for(int i=0;i<4;i++) {
							tempText[i] = tempText[i+1];
						}
						tempText[4] = text[counter+4];
						cursor = 0;
					}
				}
				EventFlag = 1;
			
			break;
			case 3:
				
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();	
				ssd1306_SetCursor(7, 66); //8x64
				ssd1306_WriteString((char *)"Wait", Font_7x10, White);
				ssd1306_SetCursor(10, 56); //11x52
				ssd1306_WriteString((char *)"BLE", Font_7x10, White);
				ssd1306_UpdateScreen();
				
				EventFlagOledOff = 8;
				oledOffCounter = 0;
				
			
				/*if(EventFlagOledOff != 8 && EventFlagOledOff == 0) {		//Oled Auto Off
					EventFlagOledOff = 7;
				}*/
				HAL_Delay(500);
				EventFlag = 1;
			
			break;
			case 4:
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();	
				ssd1306_SetCursor(7, 66);
				ssd1306_WriteString((char *)"Done", Font_7x10, White);
				ssd1306_SetCursor(10, 56);
				ssd1306_WriteString((char *)"BLE", Font_7x10, White);
				ssd1306_UpdateScreen();
			
				HAL_Delay(2000);
				page = 0;
				//EventFlag = 2; //Set 8+2 8 für auto display off und 2 zum display aktualisieren
			break;
			case 5:
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();	
				ssd1306_SetCursor(11, 62);
				ssd1306_WriteString((char *)"ON", Font_7x10, White);
				ssd1306_UpdateScreen();

				HAL_Delay(2000);
				if(bleConnected) {
					page = 0;
				} else {
					page = 3;
				}
				break;
			default:
				
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();	
				//time
				ssd1306_SetCursor(6, 58); //8x64
				ssd1306_WriteChar(time[0], Font_6x8, White);
				ssd1306_WriteChar(time[1], Font_6x8, White);
				ssd1306_SetCursor(17, 58); //8x64
				ssd1306_WriteChar(':', Font_6x8, White);
				ssd1306_SetCursor(21, 58); //8x64
				ssd1306_WriteChar(time[2], Font_6x8, White);
				ssd1306_WriteChar(time[3], Font_6x8, White);
				//date
				ssd1306_SetCursor(6, 66); //8x64
				ssd1306_WriteChar(date[0], Font_6x8, White);
				ssd1306_WriteChar(date[1], Font_6x8, White);
				ssd1306_SetCursor(17, 66); //8x64
				ssd1306_WriteChar('.', Font_6x8, White);
				ssd1306_SetCursor(21, 66); //8x64
				ssd1306_WriteChar(date[2], Font_6x8, White);
				ssd1306_WriteChar(date[3], Font_6x8, White);
				ssd1306_UpdateScreen();
				EventFlag = 1;
				
				
	
			break;			
		}
	}
}

void thread_oled_auto_off(void* arg){
		
	if(oledOffCounter > 0) {
		oledOffCounter++;
	}
	
	if(EventFlagOledOff == 8 && oledOffCounter == 0) {
		oledOffCounter = 1;
		activindicator = EventFlag;
	}
	/*else if(EventFlagOledOff == 7 && oledOffCounter == 0) {
		oledOffCounter = 1;
		activindicator = EventFlag;
	}*/

	if(!(activindicator & 8) && oledOffCounter > 25000 && !oled_off_merker) {
		ssd1306_SetDisplayOn(0);
		HAL_Delay(3);
		HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_RESET);
		oled_off_merker = 1;
		pagemerker = page;
		page = 0;
		activindicator = 0;
		oledOffCounter = 0;
		EventFlagOledOff = 0;
		//osTimerStop(messageroll);
	} 
}

void thread_pageclick(){
	if(EventFlag == 4){
		
		switch(page){
			case 0:
			CAP1203_Set_Standby_CS(CS2_ENB);
			CAP1203_Set_PWR_CS(PWR_CS2);
			CAP1203_Set_PWR_Time(PWR_TIME_2240_MS);
			CAP1203_Go_PWR();
			//CAP1203_Int_Clr();
			
			ssd1306_Fill(Black);
			ssd1306_SetCursor(11, 62);
			ssd1306_WriteString((char *)"OFF", Font_7x10, White);
			ssd1306_UpdateScreen();
			
			oled_off_merker = 1;
			oledOffCounter = 0;
			EventFlagOledOff = 0;
//			HAL_NVIC_DisableIRQ(EXTI3_IRQn);
//			osThreadSuspend(id_data_sync);
//			osThreadSuspend(id_oled_data);
//			osThreadSuspend(id_touchdetection);
			powerOff = 1;
			CAP1203_Int_Clr();
			HAL_Delay(2000);	
			//ssd1306_Fill(Black);
			//ssd1306_UpdateScreen();
			ssd1306_SetDisplayOn(0);
			HAL_Delay(3);
			
			HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_RESET);
			
			LL_PWR_EnterLowPowerRunMode();
//			osKernelSuspend();
//			
//			LL_C2_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN1);
//			LL_PWR_IsEnabledWakeUpPin(LL_PWR_WAKEUP_PIN1);
//			LL_PWR_SetWakeUpPinPolarityLow(LL_PWR_WAKEUP_PIN1);
//			
//			loeschen = LL_PWR_IsWakeUpPinPolarityLow(LL_PWR_WAKEUP_PIN1);
			/*HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_LOW);
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE); */
			
			
			
			/*if(   (LL_PWR_IsActiveFlag_C1SB() == 0) || (LL_PWR_IsActiveFlag_C2SB() == 0)) {
				// Set the lowest low-power mode for CPU2: shutdown mode 
				LL_C2_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
			}
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			HAL_PWR_EnterSTANDBYMode(); */
			break;
			
			default:
				//nothing
			break;
		}	
	} 
}

void touchhelper() {
	cs_oldvalue = 0;
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);  
}
void pageclickhelper() {
	for(int i = 0; i <= filterwert; i++){
	cs_val[i] = 0;
	}
}

void timeupdate() {
	//add RTC implementation
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL EventFlag return state */
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

