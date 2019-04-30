-----------------------------------------------------------*/
#include "main.h"

#define channel2  ADC_CHANNEL_3
#define channel6  ADC_CHANNEL_6
#define channel7  ADC_CHANNEL_7
---------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

//PWM 
float pulse_width=200;
float last_pulse = 200;
float duty = 0;
int Stop_Flag = GPIO_PIN_RESET;

//WIND SENSING ADC
int32_t POT_ADC= 0;

//POWER SENSING ADC
int32_t Voltage_ADC = 0;
int32_t Current_ADC = 0;

//POWER VARIABLES
float voltage = 0;
float voltagetemp = 0;
float current = 0;
float currenttemp = 0;
float power = 0;

//AVERAGE POWER VARIABLES
float old_power =0;
float powertemp =0;
int count = 0;
int count2 =0;

//STEPPER VARIABLES 
int DIR = 0;
int step = 0;

//PID
float PID_Targed = 1700.0;
float errors= 0;
//P
float kp = 1.0;
float Perror = 0;
//I
float i = 0.1;
float intError = 0;
float totalError = 0;

//D
float d= 0.01;
float diffError = 0;
float lastError= 0;

//PID OUTPUT
float PID_output= 0;

//STRING FOR UART
uint8_t recieve[10];
uint32_t recieve_len = 5;
uint8_t str[50];
uint32_t str_len = 0;



void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
float calc_error(int32_t);

//SELECT AND READ AN ADC CHANNELL
void config_ext_channel_ADC(uint32_t);
uint32_t r_single_ext_channel_ADC(uint32_t);

//PWM FUNCTIONS
void update_pwm(float);
void calc_pwm_update(float , float);

//STEPPER FUNCTIONS
void update_step();
void stepper(int);


int main(void)
{

  HAL_Init();

  
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  
  //TIMER 1 START CONTROLS SPEED OF STEPPER
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  
  //TIMER 2 START FOR READING WINDSENSOR 
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  
  //TIMER 3 START PWM FOR BOOST CONVERTER
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);    
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
  
  //TIMER 4 START FOR MPPT
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  
  //UART START
  HAL_UART_Init(&huart1);
  while (1)
  {
  }
}

/* USER CODE BEGIN 4 */
//Interrupt call back function for timers
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//If timer 1 interrupts we will update the stepper mottor
	if(htim->Instance == TIM1)
	{	
		//stepper(1);
		HAL_GPIO_TogglePin(GPIOB, STEP_Pin);
	}
	
	//If timer 2 interrupts we read the wind sensor and determine what we must do with the stepper
	if(htim->Instance == TIM2)
	{
    POT_ADC= r_single_ext_channel_ADC(channel2);
    update_step();
	}
	
	//If timer 4 interrupts we read the power sensors and calculate an average.
	//Every 100 reads we take the average for mppt and send the data over UART 
	if(htim->Instance == TIM4)
	{
	    Stop_Flag = HAL_GPIO_ReadPin(GPIOB, PI_IN_Pin);
		//Read the ADC for voltage and determine its value 
	    Voltage_ADC = r_single_ext_channel_ADC(channel7);
	    voltage =  ((float)Voltage_ADC/4096.0)*3.3*4.85;
		
		//Read the ADC for current and determine its value
	    Current_ADC = r_single_ext_channel_ADC(channel6);
	    current = ((float)Current_ADC/4096.0)*3.3/(373.0*0.005*2.0);
		
		//Calculate the power then sum it for averaging
	    powertemp = voltage * current;
	    power += powertemp;
	    voltagetemp += voltage;
	    currenttemp += current;
		
		//Every 100 samples we will average
	    count++;
	    if(count == 1000)
	    {
		//Average the power and reset count
	    count = 0;
	    power = power/1000.0;
	    voltage = voltagetemp/1000.0;
	    current = currenttemp/1000.0;
		//count2 ++;

		//Check if we have recieved a digita stop signal
	    if(Stop_Flag == GPIO_PIN_RESET)
	    {
			//if are voltage is too low we need to let the generator spin up freely
	    	if(voltage < 8)
	    	{
	    	pulse_width = 200;
	    	last_pulse = 20;
	    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	    	HAL_GPIO_WritePin(GPIOB, LOAD_DC_Pin, GPIO_PIN_SET);
	    	}
	    	else //otherwuse we use mppt to control pwm
	    	{
	    	calc_pwm_update(old_power, power);
	    	HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOB, LOAD_DC_Pin, GPIO_PIN_RESET);
	    	}
	    }
	    else //if we have a stop signal we will overload the generator to slow it down
	    {
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1084);
	    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
	   	  HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
	    }

	    old_power = power;
		//Prepare the power value for UART transmission
				 duty = ((float)pulse_width*100.0)/1085.0;
				count2 = 0;
				str_len = sprintf(str, "P%.6f V%.6f C%.6f D%.2f\n",power,voltage,current,duty);
				HAL_UART_Transmit_DMA(&huart1, str, (uint32_t)str_len);

		 voltagetemp = 0;
		 currenttemp = 0;
		 power=0;
	    }
	    }
	}

//Function to determine stepper direction and speed
void update_step()
{
	//Covert the output from PID to proper prescale values to be used to control timer
	PID_output = 15000.0/(calc_error(POT_ADC));
	if(PID_output <= 10 && PID_output >= 0)
	PID_output = 10;
	else if(PID_output >= -10 && PID_output < 0)
	PID_output = -10;
	//First output will stop the motor if we are recieving values outside our bounds
	if(PID_output > 500 || PID_output < -500)
	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_SET);
	//Determine which direction we need to spin the motor and change its speed
	else if(PID_output > 0)
	{
	HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_PRESCALER(&htim1, (int32_t)PID_output);
	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_RESET);
	}
	else
	{
	HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_RESET);
	PID_output = PID_output * -1;
	__HAL_TIM_SET_PRESCALER(&htim1, (int32_t)PID_output);
	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_RESET);
	}
	
}

//Function used to calculate PID for motor control 
float calc_error(int32_t feedback){
	//Store the previous value 
	lastError = errors;
	
	//Calculate the new error
	errors = (feedback - PID_Targed);
	Perror = errors*kp;
	
	//Only begin calculating the integral error as we approach the desired value
	//This helps us avoid the integral from exploding early 
	if(errors < 300 && errors > -300)
	totalError = 0;
	else if(errors < 800 && errors >-800)
	totalError += errors;
	else
	totalError = 0;
	//Calculate I error 
	intError = totalError * i;
	
	//Calulate D error and set it to zero if we are close.
	if(errors < 300 && errors > 300)
	diffError = 0;
	else
	diffError = (errors - lastError) * d;

	return Perror + intError + diffError;
}

//Function to determine what to do to BOOST PWM for MPPT
//perturb and observe controller 
void calc_pwm_update(float last, float current)
{
	//If we lay outside of desired bounds we force the pwm to react 
if(voltage <= 10)
update_pwm(-10.0);
else if(voltage >= 15)
update_pwm(10.0);
//If we had an increase in power we will continue to move the PWM in the same direction 
if(current > last)
    {
    	if(pulse_width > last_pulse)
    	{
    		update_pwm(10.0);
    	}
    	else
    	{
    		update_pwm(-10.0);
    	}
    }
//If we decreased power we will do the opposite to the PWM
else 
	{
		if(pulse_width > last_pulse)
		{
			update_pwm(-10.0);
		}
		else
		{
			update_pwm(10.0);
		}
	}
}

//Function to change the PWM for Boost 
void update_pwm(float change)
{
	//Check if the update will move us past desired values
	if((pulse_width + change) <= 650)// && pulse_width >= 320 )
	{
	//Store the last value then update the pulse width 
	last_pulse = pulse_width;
	pulse_width = pulse_width + change;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (int)pulse_width);
	}
}

//Functions that set the ADC channel and reads the values
void config_ext_channel_ADC(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig;

  sConfig.Channel = channel;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;


    sConfig.Rank = 1;


  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}
//Functions that set the ADC channel and reads the values
uint32_t r_single_ext_channel_ADC(uint32_t channel)
{
  uint32_t digital_result;

  config_ext_channel_ADC(channel);

  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  digital_result = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  config_ext_channel_ADC(channel);

  return digital_result;
}

//Foloowing commands are generated by stmcube
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 6400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1024-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 6400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_DC_GPIO_Port, LOAD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEP_Pin|DIR_Pin|EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LOAD_DC_Pin */
  GPIO_InitStruct.Pin = LOAD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOAD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_Pin DIR_Pin EN_Pin */
  GPIO_InitStruct.Pin = STEP_Pin|DIR_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PI_IN_Pin */
  GPIO_InitStruct.Pin = PI_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PI_IN_GPIO_Port, &GPIO_InitStruct);

}

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
