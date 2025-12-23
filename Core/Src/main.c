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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "usbd_cdc_if.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
#define MPU6050_ADDR 0xD0
int16_t Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW;
float Ax, Gx;
float Angle_Balance = 0;

float Kp_Ang = 10, Ki_Ang =  0.1, Kd_Ang = 0.34;
float Setpoint_Ang = 2;
float Error_Ang = 0, Last_Error_Ang = 0, Integral_Ang = 0;
float Target_Speed = 0;

float Kp_Spd = 0.93, Ki_Spd = 0.36, Kd_Spd = 0;
float Current_Speed = 0;
float Error_Spd = 0, Integral_Spd = 0, Last_Error_Spd = 0;
float PWM_Out = 0;

float count1 = 0;
float count2 = 0;
char buffer[64];
#define M1_IN1_PORT GPIOC
#define M1_IN1_PIN  GPIO_PIN_0
#define M1_IN2_PORT GPIOC
#define M1_IN2_PIN  GPIO_PIN_4
#define M1_TIMER_PWM &htim2
#define M1_TIMER_CHANNEL TIM_CHANNEL_1
#define M2_IN3_PORT GPIOC
#define M2_IN3_PIN  GPIO_PIN_2
#define M2_IN4_PORT GPIOC
#define M2_IN4_PIN  GPIO_PIN_3
#define M2_TIMER_PWM &htim2
#define M2_TIMER_CHANNEL TIM_CHANNEL_2

#define PWM_MAX_VALUE 999
volatile uint8_t flag_run_pid = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t g_usb_rx_buffer[64];
extern volatile uint8_t g_usb_rx_flag;
void USB_SendString(char* str)
{
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
}
void MPU6050_Init(void) {
    uint8_t check = 0;
    uint8_t data;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);

    if (ret != HAL_OK) {
        USB_SendString("Error: I2C Connect Failed! Check wires.\r\n");
    }
    else if (check == 0x68) {
        data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 1000);

        USB_SendString("MPU6050 Init: OK (Found 0x68)\r\n");
    } else {
        char msg[64];
        sprintf(msg, "Error: Wrong ID! Read: 0x%02X (Expected 0x68)\r\n", check);
        USB_SendString(msg);
    }
}

void Read_Sensor(void) {
    uint8_t Rec_Data[14];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, 1000);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    Ax = atan2(Accel_Y_RAW, Accel_Z_RAW) * 57.296;
    Gx = Gyro_X_RAW / 131.0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5) {
	      flag_run_pid = 1;
	  }
}

void Control_Motor(int motor_num, char direction, int percentage)
{
  if (percentage < 0)   percentage = 0;
  if (percentage > 100) percentage = 100;

  if (direction == 'S') {
	  percentage = 0;
  }
  uint32_t pwm_value = (uint32_t)(percentage * PWM_MAX_VALUE / 100.0);
  if (motor_num == 1)
  {
      if (direction == 'F')
      {
          HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, GPIO_PIN_RESET);
      }
      else if (direction == 'R')
      {
          HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, GPIO_PIN_SET);
      }
      else
      {
          HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, GPIO_PIN_RESET);
      }

      __HAL_TIM_SET_COMPARE(M1_TIMER_PWM, M1_TIMER_CHANNEL, pwm_value);
  }
  else if (motor_num == 2)
  {
      // Đặt chiều quay
      if (direction == 'F')
      {
          HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, GPIO_PIN_RESET);
      }
      else if (direction == 'R')
      {
          HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, GPIO_PIN_SET);
      }
      else
      {
          HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, GPIO_PIN_RESET);
      }
      __HAL_TIM_SET_COMPARE(M2_TIMER_PWM, M2_TIMER_CHANNEL, pwm_value);
  }
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
   HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
   HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
   MPU6050_Init();
      HAL_Delay(50);
   HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (flag_run_pid == 1) {
	  flag_run_pid = 0;
	  float dt = 0.005;
	  	        Read_Sensor();
	  	        Angle_Balance = 0.98 * ((float)Angle_Balance + (float)Gx * dt) + 0.02 * (float)Ax;
	  	        Error_Ang = Angle_Balance - Setpoint_Ang;
	  	        Integral_Ang += Error_Ang * dt;
	  	        if (Integral_Ang > 100) Integral_Ang = 100;
	  	        if (Integral_Ang < -100) Integral_Ang = -100;
	  	        float Derivative_Ang = (Error_Ang - Last_Error_Ang) / dt;
	  	        Last_Error_Ang = Error_Ang;
	  	        Target_Speed = (Kp_Ang * Error_Ang) + (Ki_Ang * Integral_Ang) + (Kd_Ang * Derivative_Ang);
	  		  if (Target_Speed > 100) Target_Speed = 100;
	  		  if (Target_Speed < -100) Target_Speed = -100;
	  	        count1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	  	        count2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
	  	        __HAL_TIM_SET_COUNTER(&htim3, 0);
	  	        __HAL_TIM_SET_COUNTER(&htim4, 0);
	  	        Current_Speed = (float)(count1 + count2) / 2.0;
	  	        Error_Spd = Target_Speed - Current_Speed;
	  	        Integral_Spd += Error_Spd * dt;
	  	        if (Integral_Spd > 200) Integral_Spd = 200;
	  	        if (Integral_Spd < -200) Integral_Spd = -200;
	  	        PWM_Out = (Kp_Spd * Error_Spd) + (Ki_Spd * Integral_Spd);
	  	        if (Angle_Balance > 80 || Angle_Balance < -80) {
	  	            PWM_Out = 0;
	  	            Integral_Ang = 0;
	  	            Integral_Spd = 0;
	  	            Last_Error_Ang = 0;
	  	            Last_Error_Spd = 0;
	  	            Target_Speed = 0;
	  	      }
	  	        int final_pwm = 0;
	  	        char dir = 'S';
	  	        if (PWM_Out > 0) {
	  	            dir = 'F';
	  	            final_pwm = (int)(PWM_Out);
	  	        } else if (PWM_Out < 0) {
	  	            dir = 'R';
	  	            final_pwm = (int)(-PWM_Out);
	  	        }
	  	      if (final_pwm > 100) final_pwm = 100;
	  	      if (final_pwm < -100) final_pwm = -100;
 	  	        if (final_pwm > 0 && final_pwm < 20) final_pwm = 30;
	  	        if (final_pwm < 0 && final_pwm > -20) final_pwm = -30;
	  	        Control_Motor(1, dir, final_pwm);
	  	        Control_Motor(2, dir, final_pwm);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (g_usb_rx_flag == 1)
	      {
	          g_usb_rx_flag = 0;
	          char type;
	          float value;
	          if (sscanf((char*)g_usb_rx_buffer, "%c %f", &type, &value) == 2)
	          {
	        	  switch(type) {
	        	  				  case 'A': case 'a': Kp_Ang = value; break;
	        	  				  case 'B': case 'b': Ki_Ang = value; break;
	        	  				  case 'C': case 'c': Kd_Ang = value; break;
	        	  				  case 'S': case 's': Setpoint_Ang = value; break;
 D
	        	  				  case 'P': case 'p': Kp_Spd = value; break;
	        	  				  case 'I': case 'i': Ki_Spd = value; break;
	        	  				  case 'D': case 'd': Kd_Spd = value; break;
	        	  			  }
	        	  			  char msg[64];
	              sprintf(msg, "SET: %c = %.2f\r\n", type, value);
	              USB_SendString(msg);
	              Error_Ang = 0; Integral_Ang = 0; Last_Error_Ang = 0;
	              Error_Spd = 0; Integral_Spd = 0; Last_Error_Spd = 0;
	          }
	      }

	      static uint32_t last_tick = 0;
	      if (HAL_GetTick() - last_tick > 100)
	      {
	          last_tick = HAL_GetTick();
	          char debug_msg[64];
	          sprintf(debug_msg, "Ang:%.2f, Speed:%.2f, PWM:%1f\r\n", Angle_Balance, Current_Speed, PWM_Out);
	          USB_SendString(debug_msg);
	      }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M1_IN1_Pin|M2_IN3_Pin|M2_IN4_Pin|M1_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M1_IN1_Pin M2_IN3_Pin M2_IN4_Pin M1_IN2_Pin */
  GPIO_InitStruct.Pin = M1_IN1_Pin|M2_IN3_Pin|M2_IN4_Pin|M1_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
