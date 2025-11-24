/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "Myservo.h"
#include "my_1602.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LCD_SPEED_INDEX (uint8_t)8
#define LCD_RAIN_INDEX (uint8_t)7

#define X_CHANNEL ADC_CHANNEL_0
#define Y_CHANNEL ADC_CHANNEL_1
#define WATER_CHANNEL ADC_CHANNEL_10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Start_ADC_Conversion(uint32_t channel);
void Print_Value(uint32_t value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	
	// ---------------------------------------------------------------JoyStick Start
	
	uint16_t adc_y_val = 0;
	uint16_t x_val = 0;
	uint16_t y_val = 0;
	uint8_t  current_channel = 0; // 0: X, 1: Y
	uint8_t joy_button = 0;

	volatile uint32_t prev_adc_x_val = -1;
	volatile uint32_t prev_adc_y_val = -1;

	// ---------------------------------------------------------------JoyStick end
	
	// ---------------------------------------------------------------Water Sensor Start
	
	const uint16_t MIN[4] = {0, 600, 1200, 1800};
	const uint16_t MAX[4] = {400, 900, 1500, 2100};
	volatile uint32_t autoMode = 0;
	volatile uint32_t mode = 0;
	uint32_t prev_range = 0;
	uint16_t water_val = 0;

	// ---------------------------------------------------------------Water Sensor End
	
	// ---------------------------------------------------------------Servo Start
	
	
	Servo myservo1;
	Servo myservo2;
	volatile uint16_t angle = 0; 
	volatile uint8_t iDirection = 0;
	volatile uint8_t isWorking = 0;
	// ---------------------------------------------------------------Servo End
	
	
	// ---------------------------------------------------------------Flag
	typedef enum {STOP = 0, LOW, MID, HIGH, AUTO } Progress; // imsi
	// ---------------------------------------------------------------Flag
	
	// ---------------------------------------------------------------LCD Start
	volatile uint8_t lcd_update_flag = 0;
	char strLCDSpeed[] = "Speed : ";
	char strLCDRain[] = "Rain : ";
	char *speedStr[5] = {"Stop", "Low", "Mid", "High", "Auto"};
	uint8_t rainChar[8] = {
		0b00100,  
		0b00100,  
		0b01010,  
		0b01010,  
		0b10001,  
		0b10001,  
		0b01110,  
		0b00000   
	};
	
	uint8_t emptyChar[8] = {
		0b00000,  
		0b00000,  
		0b00000,  
		0b00000,  
		0b00000,  
		0b00000,  
		0b00000,  
		0b00000   
	};
	
	// ---------------------------------------------------------------LCD End
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
  // ------------------------------------------------------------- joystic init Start
	current_channel = 0; 
  Start_ADC_Conversion(X_CHANNEL); 
	uint16_t prev_x_val = -1;
	uint16_t prev_y_val = -1;
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0); // 높은 우선순위
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  // ------------------------------------------------------------- joystic init End
  
  // ------------------------------------------------------------- water Sensor init Start
	HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_ADC_Start_IT(&hadc1);
	// ------------------------------------------------------------- water Sensor init End
	
	// ------------------------------------------------------------- servo init Start 

	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	init_Servo(&myservo1, TIM_CHANNEL_1, &htim1);
	init_Servo(&myservo2, TIM_CHANNEL_2, &htim1);
	
	myservo1.Set_Range(&myservo1,0,180,500,2500);
	myservo2.Set_Range(&myservo2,0,180,500,2500);

	
	HAL_TIM_Base_Start_IT(&htim1);
	// --------------------------------------------------------------- servo init End

	// --------------------------------------------------------------- LCD init Start
	
	HAL_Delay(1000); // imsi
	lcd_init(&hi2c1);
	
	lcd_send_string(strLCDSpeed);
	lcd_set_cursor(0,LCD_SPEED_INDEX);
	lcd_send_string("Stop");
	HAL_Delay(1000);
	lcd_create_char(0,rainChar);
	lcd_create_char(1,emptyChar);
	
	// --------------------------------------------------------------- LCD init Start
	
	Servo myservo3;
  
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		char msg[50] = {0};
		
		if(x_val != prev_x_val)
		{
			if(x_val > 2500)
			{
				mode = 3;
				isWorking = 1;
				lcd_update_flag = 1;
				sprintf(msg, "HIGH");
				myservo1.Set_angle(&myservo1,angle);
				myservo2.Set_angle(&myservo2,angle);
			}
				
			else if(x_val < 1500)
			{
				mode = 4;
				sprintf(msg, "AUTO");
				lcd_update_flag = 1;
				
				 HAL_ADC_Stop_IT(&hadc1);
         Start_ADC_Conversion(WATER_CHANNEL);
				myservo1.Set_angle(&myservo1,angle);
				myservo2.Set_angle(&myservo2,angle);
			}
			else
				;
				
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			prev_x_val = x_val;
		}
			
		if(y_val != prev_y_val)
		{
			if(y_val > 2500)
			{
				mode = 2;
				isWorking = 1;
				lcd_update_flag = 1;
				sprintf(msg, "NORMAL");
				myservo1.Set_angle(&myservo1,angle);
				myservo2.Set_angle(&myservo2,angle);
			}
			
			else if(y_val < 1500)
			{
				mode = 1;
				isWorking = 1;
				lcd_update_flag = 1;
				sprintf(msg, "LOW");
				myservo1.Set_angle(&myservo1,angle);
				myservo2.Set_angle(&myservo2,angle);
			}
			else
				;
				
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			prev_y_val = y_val;
		}
		
		if(joy_button)
		{
			mode = 0;
			sprintf(msg, "STOP");
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			joy_button = 0;
		}
		
		if(lcd_update_flag)
		{
				lcd_replace_char(0,LCD_SPEED_INDEX, LCD_SPEED_INDEX+5,0x01);
				lcd_set_cursor(0,LCD_SPEED_INDEX);
			  lcd_send_string(speedStr[mode]);
				HAL_Delay(10);
				lcd_set_cursor(1,0);
				lcd_send_string(strLCDRain);
			
				lcd_set_cursor(1,LCD_RAIN_INDEX);
				lcd_replace_char(1,LCD_RAIN_INDEX, LCD_RAIN_INDEX + 3,0x00);
				
				lcd_replace_char(1,(LCD_RAIN_INDEX + autoMode), LCD_RAIN_INDEX + 3, 0x01);
				
				if(mode != AUTO){
					lcd_replace_char(1,0,16,0x01);
				}
				
				lcd_update_flag = 0;
		}
		HAL_Delay(100);
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 400;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Print_Value(uint32_t value)
{
    char msg[50];
    if(value >= MIN[0] && value <= MAX[0])
    {
        if(prev_range != 0)
        {
            sprintf(msg, "STOP %u\n", value);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            prev_range = 0;
            autoMode = 0;
						lcd_update_flag = 1;
						isWorking = 0;
        }
    }
    else
    {
        for(uint8_t i = 1; i < 4; i++)
        {
            if(value >= MIN[i] && value <= MAX[i])
            {
                if(prev_range != i)
                {
                    switch(i)
                    {
                        case 1:
                            sprintf(msg,"LOW %u\n", value);
                            autoMode = 1;
                            break;
                    
                        case 2:
                            sprintf(msg,"NORMAL %u\n", value);
                            autoMode = 2;
                            break;
                                
                        case 3:
                            sprintf(msg,"HIGH %u\n", value);
                            autoMode = 3;
                            break;
                    }
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    prev_range = i;
										lcd_update_flag = 1;
										isWorking = 1;
                }
                break;
            }
        }
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint16_t val = HAL_ADC_GetValue(hadc);

    if (mode == 4) // AUTO ����� �� ���� ����
    {
        water_val = val;
        Print_Value(val);

        // Watchdog ���� �缳��
        ADC_AnalogWDGConfTypeDef awd = {0};
        awd.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
        awd.HighThreshold = MAX[autoMode];
        awd.LowThreshold = MIN[autoMode];
        awd.Channel = WATER_CHANNEL;
        awd.ITMode = ENABLE;
        HAL_ADC_AnalogWDGConfig(hadc, &awd);
				
				Start_ADC_Conversion(WATER_CHANNEL);
    }
    else // MODE != 0 �� ���̽�ƽ ó��
    {
        if(current_channel == 0)
        {
            x_val = val;
            current_channel = 1;
            Start_ADC_Conversion(Y_CHANNEL);
        }
        else
        {
            y_val = val;
            current_channel = 0;
            Start_ADC_Conversion(X_CHANNEL);
        }
    }
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1 && mode == 0)
    {
        Print_Value(water_val);
    }
}

void Start_ADC_Conversion(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
		sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);    
   
    HAL_ADC_Start_IT(&hadc1);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        joy_button = 1;
				isWorking = 0;
				lcd_update_flag = 1;
    }
}

	
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

		if(htim->Instance == TIM1)
		{
			if((mode+autoMode) % 4 == 0){
					angle = 0;
			}
			
			if(isWorking)
			{
				uint8_t gap = 0;
				
					switch((mode+autoMode) % 4)
					{
						case LOW :
							gap = 4;
							break;
						case MID:
							gap = 7;
							break;
						case HIGH:
							gap = 9;
						break;
					}
				
					if(iDirection)
					{ 
						if( angle >= gap)
							angle -= gap;
						else 
							angle = myservo1.Min_Angle;
					}
					else{
						angle += gap;
					}
				
				
					if(angle >= myservo1.Max_Angle) // 방향 전환
						iDirection = 1;
					else if( angle <= myservo1.Min_Angle)
						iDirection = 0;
			}
			myservo1.Set_angle(&myservo1,angle);
			myservo2.Set_angle(&myservo2,angle);

		}
		// else if(htim->Instance == TIM2)
		// {
		// 	if(lcd_update_flag)
		// 	{
		// 		lcd_update_flag = 0;
		// 	}
		// }
	
	
}

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
