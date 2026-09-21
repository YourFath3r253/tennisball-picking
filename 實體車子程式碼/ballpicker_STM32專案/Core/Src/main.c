/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float target_rpm;  // 目標轉速
    float actual_rpm;  // 實際轉速
    float error;       // 當前誤差
    float error_last;  // 上次誤差
    float integral;    // 誤差積分 (I項累積)

    float Kp;          // 比例參數
    float Ki;          // 積分參數
    float Kd;          // 微分參數

    float out_max;     // PWM 輸出上限 (例如 1000)
} PID_Controller;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_byte;          // 儲存每次接收到的 1 個位元組
uint8_t rx_buffer[100];   // 接收緩衝區
volatile uint8_t rx_index = 0;     // 緩衝區索引
volatile uint8_t command_ready = 0;// 封包接收完成標誌
// 🚀 新增這行：宣告防卡球狀態變數，讓下面的 if (dejam_active == 0) 可以順利執行！
uint8_t dejam_active = 0;
// 🚀 新增：用來儲存 Jetson 傳來的球距離 (cm) 與 角度 (度)
float ball_distance_cm = 0.0f;
float ball_angle_deg = 0.0f;
// 👇 新增：用來儲存球場邊界狀態
char court_state[10] = "SAFE";
char court_dir[10] = "NONE";
float court_outside_ratio = 0.0f;
// 宣告變數來儲存左右輪的編碼器數值
int16_t chassis_enc_left = 0;
int16_t chassis_enc_right = 0;
// 新增：儲存計算後的真實轉速 (RPM)
float chassis_rpm_left = 0.0;
float chassis_rpm_right = 0.0;
// 一階低通濾波後的轉速變數
// ==========================================
float rpm_left_filter = 0.0f;
float rpm_right_filter = 0.0f;
float pwm_cmd_left = 0.0f;
float pwm_cmd_right = 0.0f;
// 新增：馬達物理規格常數 (基本解析度 11 * 減速比 30 * 4倍頻)
const float ENCODER_RESOLUTION = 11.0f * 18.8f * 4.0f;
PID_Controller pid_left;
PID_Controller pid_right;
// 底盤機械幾何尺寸 (請依照你實際量測的尺寸修改)
const float WHEEL_RADIUS = 0.052f; // 輪胎半徑 35mm (0.035公尺)
const float WHEEL_TRACK  = 0.243f;  // 左右輪心間距 250mm (0.25公尺)

// [Sean 2026-09-21] 連續P控制轉向參數：第一次設定，還沒實機驗證過，需要調整
// error = 0 - angle；y = Kp*error（=left_rpm-right_rpm）；左=y/2、右=-y/2
const float STEER_KP = 0.6f;          // rpm / 度 [Sean 2026-09-21] 0.3已驗證乾淨收斂(run13/14平均7.6s,無overshoot)，加倍測試找overshoot邊界
const float STEER_WHEEL_MAX = 45.0f;  // 單輪rpm飽和上限，比今天驗證過安全的40rpm高一點

// 💡 里程計核心：記錄車體當前旋轉的總角度 (度數)
float chassis_angle_deg = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void Chassis_SpinLeft(float turn_rpm);
void Chassis_SpinRight(float turn_rpm);
void Chassis_SteerP(float left_rpm, float right_rpm);
void Chassis_Forward(float forward_rpm);
void Chassis_Stop(void);
void Chassis_TurnPrecise(float target_angle_change, float spin_rpm);
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // 開啟 USART2 中斷接收，每次只接收 1 個位元組，存入 rx_byte
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  // 1. 設定左輪馬達方向為正轉 (IN1=High, IN2=Low)
    HAL_GPIO_WritePin(GPIOC, L_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, L_IN2_Pin, GPIO_PIN_SET);

    // 2. 設定右輪馬達方向為正轉 (IN1=High, IN2=Low)
    HAL_GPIO_WritePin(GPIOC, R_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, R_IN2_Pin, GPIO_PIN_SET);

    // 3. 啟動定時器 2 的通道 1 (PA0) 與通道 2 (PA1)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // 4. 設定左右滾輪馬達轉速 (若 ARR 改為 1000-1，則 500 代表 50% 轉速)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 950);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 950);
    // ==========================================
    // 底盤馬達初始化設定
    // ==========================================

    // 1. 預設底盤馬達方向為正轉 (前進)
    HAL_GPIO_WritePin(GPIOC, CHASSIS_L_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, CHASSIS_L_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, CHASSIS_R_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, CHASSIS_R_IN2_Pin, GPIO_PIN_SET);

    // 2. 啟動底盤的 PWM (TIM3 的 Channel 1 與 Channel 2)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // 3. 初始轉速設為 0 (避免一開機就暴衝)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);

    // 4. 啟動底盤的編碼器讀取 (TIM4 與 TIM1)
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 左輪
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 右輪
    // 初始化左輪 PID 參數
      pid_left.Kp = 5.0f;       // P參數 (稍後需實機微調)
      pid_left.Ki = 1.0f;      // I參數
      pid_left.Kd = 0.01f;      // D參數
      pid_left.out_max = 1000.0f; // 你的 TIM3 ARR 最大值應該是 1000
      pid_left.integral = 0.0f;
      pid_left.target_rpm = 0.0f; // 先設定一個安全的目標速度：150 RPM

      // 初始化右輪 PID 參數 (可以直接複製左輪體質)
      pid_right = pid_left;

    // 5. 啟動 TIM6 的 10ms 硬體中斷
    HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
      /* USER CODE BEGIN WHILE */
      while (1)
      {
    	  // 偵測 Nucleo 板子上的藍色按鈕是否被按下 (防卡球排除機制)
    	        if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
    	        {
    	            // 左輪反轉、右輪反轉 (吐球)
    	            HAL_GPIO_WritePin(L_IN1_GPIO_Port, L_IN1_Pin, GPIO_PIN_RESET);
    	            HAL_GPIO_WritePin(L_IN2_GPIO_Port, L_IN2_Pin, GPIO_PIN_SET);
    	            HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, GPIO_PIN_SET);
    	            HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, GPIO_PIN_RESET);

    	            HAL_Delay(1000);

    	            // 恢復正轉
    	            HAL_GPIO_WritePin(L_IN1_GPIO_Port, L_IN1_Pin, GPIO_PIN_SET);
    	            HAL_GPIO_WritePin(L_IN2_GPIO_Port, L_IN2_Pin, GPIO_PIN_RESET);
    	            HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, GPIO_PIN_RESET);
    	            HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, GPIO_PIN_SET);

    	            HAL_Delay(500);
    	        }

        /* USER CODE END WHILE */

    	        /* USER CODE BEGIN 3 */
    	                        if (command_ready == 1)
    	                        {
    	                            char tx_msg[128];
    	                            int tx_len = 0;

    	                            // ==========================================================
    	                            // [Sean 2026-09-21] 連續P控制轉向（第一次設定，Kp=1.5，未實機驗證）：
    	                            //   距離>30cm -> error=0-angle, y=Kp*error, 左=y/2 右=-y/2（各自飽和±45rpm）
    	                            //   距離<=30cm -> 停止
    	                            //   每個vision frame都直接設定target_rpm，不清積分（跟舊的離散版本不同）
    	                            // ==========================================================
    	                            if (strncmp((char *)rx_buffer, "BALL,", 5) == 0)
    	                            {
    	                                if (sscanf((char *)rx_buffer, "BALL,%f,%f", &ball_distance_cm, &ball_angle_deg) == 2)
    	                                {
    	                                    tx_len = sprintf(tx_msg, "ACK:BALL,%.1f,%.1f\n", ball_distance_cm, ball_angle_deg);
    	                                    HAL_UART_Transmit(&huart2, (uint8_t*)tx_msg, tx_len, 50);

    	                                    if (ball_distance_cm > 30.0f)
    	                                    {
    	                                        float error = 0.0f - ball_angle_deg;
    	                                        float y = STEER_KP * error; // y = left_rpm - right_rpm

    	                                        float half_y = y / 2.0f;
    	                                        if (half_y > STEER_WHEEL_MAX)  half_y = STEER_WHEEL_MAX;
    	                                        if (half_y < -STEER_WHEEL_MAX) half_y = -STEER_WHEEL_MAX;

    	                                        Chassis_SteerP(-half_y, half_y); // [Sean 2026-09-21] 方向反了，對調左右
    	                                    }
    	                                    else
    	                                    {
    	                                        Chassis_Stop();
    	                                    }
    	                                }
    	                                else
    	                                {
    	                                    HAL_UART_Transmit(&huart2, (uint8_t*)"ERR:SCANF_FAILED\n", 17, 50);
    	                                }
    	                            }
    	                            else if (strncmp((char *)rx_buffer, "NOBALL", 6) == 0 || strncmp((char *)rx_buffer, "BALL_OFF", 8) == 0)
    	                            {
    	                                HAL_UART_Transmit(&huart2, (uint8_t*)"ACK:NOBALL\n", 11, 50);
    	                                Chassis_Stop();
    	                            }
    	                            else
    	                            {
    	                                tx_len = sprintf(tx_msg, "ERR:UNKNOWN,%s", (char*)rx_buffer);
    	                                HAL_UART_Transmit(&huart2, (uint8_t*)tx_msg, tx_len, 50);
    	                            }

    	                            // 清空快取
    	                            memset(rx_buffer, 0, sizeof(rx_buffer));
    	                            rx_index = 0;
    	                            command_ready = 0;
    	                        }
    	                            /* USER CODE END 3 */
}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, L_IN1_Pin|L_IN2_Pin|R_IN1_Pin|R_IN2_Pin
                          |CHASSIS_L_IN1_Pin|CHASSIS_L_IN2_Pin|CHASSIS_R_IN1_Pin|CHASSIS_R_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L_IN1_Pin L_IN2_Pin R_IN1_Pin R_IN2_Pin
                           CHASSIS_L_IN1_Pin CHASSIS_L_IN2_Pin CHASSIS_R_IN1_Pin CHASSIS_R_IN2_Pin */
  GPIO_InitStruct.Pin = L_IN1_Pin|L_IN2_Pin|R_IN1_Pin|R_IN2_Pin
                          |CHASSIS_L_IN1_Pin|CHASSIS_L_IN2_Pin|CHASSIS_R_IN1_Pin|CHASSIS_R_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// 🚀 請用這段最新帶有「積分分離」的函式，整段覆蓋掉你原本舊的 PID_Calc
float PID_Calc(PID_Controller *pid, float current_rpm) {
    // 1. 計算誤差
    pid->actual_rpm = current_rpm;
    pid->error = pid->target_rpm - pid->actual_rpm;

    // ==========================================
    // 2. 🚀 升級：按目標轉速比例的「動態積分分離」 🚀
    // ==========================================
    // 計算當前目標轉速的 20% 作為動態門檻 (加上絕對值處理，確保反轉也可用)
    float dynamic_limit = pid->target_rpm * 0.50f;
    if (dynamic_limit < 0) dynamic_limit = -dynamic_limit;

    // 如果低速目標算出來的門檻太小（例如目標設 10 RPM 時門檻變 2），設定一個最低保障門檻 10.0f
    if (dynamic_limit < 30.0f) dynamic_limit = 30.0f;

    // 只有當誤差進入目標轉速的動態範圍內，才允許累積積分
    if (pid->error < dynamic_limit && pid->error > -dynamic_limit)
    {
        pid->integral += pid->error;

        // 積分防暴衝限幅限制
        if (pid->integral > 8000.0f)  pid->integral = 8000.0f;
        if (pid->integral < -8000.0f) pid->integral = -8000.0f;
    }
    else
    {
        // 誤差還很大時，果斷清空歷史包袱，防止原地的積分飽和
        pid->integral = 0.0f;
    }

    // 3. 計算微分項 (D)
    float derivative = pid->error - pid->error_last;

    // 4. 計算總輸出
    float output = (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 5. 更新歷史誤差
    pid->error_last = pid->error;

    // 6. 輸出限幅
    if (output > pid->out_max) output = pid->out_max;
    if (output < -pid->out_max) output = -pid->out_max;

    return output;
}
// 當 TIM6 (10ms) 時間到時，會自動跳進這個函式
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        // 1. 讀取這 10ms 內累積的脈衝數 (底盤左右輪)
        chassis_enc_left = -((int16_t)__HAL_TIM_GET_COUNTER(&htim4));
        chassis_enc_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);

        // 2. 讀完立刻歸零！(這是 M 法測速的核心，你原本漏掉這步)
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        __HAL_TIM_SET_COUNTER(&htim1, 0);

        // (之後要把脈衝數換算成 RPM 的公式，以及 PID 運算都加在這裡)
        // 3. 換算成真實轉速 RPM ( 脈衝數 / 總解析度 * 時間放大倍率 )
        chassis_rpm_left = ((float)chassis_enc_left / ENCODER_RESOLUTION) * 6000.0f;
        chassis_rpm_right = ((float)chassis_enc_right / ENCODER_RESOLUTION) * 6000.0f;
        // 導入口低通濾波器，平滑跳動的訊號
                // ==========================================
                rpm_left_filter = (0.8f * rpm_left_filter) + (0.2f * chassis_rpm_left);
                rpm_right_filter = (0.8f * rpm_right_filter) + (0.2f * chassis_rpm_right);
        // 將濾波後的乾淨速度餵給 PID 大腦
                // ========================================================
                        // 🚀 新增：里程計 (Odometry) 角度積分推算
                        // ========================================================
                        // (a) 算這 10ms 內左右輪各自走的直線距離 (公尺)
                        float dist_l = ((float)chassis_enc_left / ENCODER_RESOLUTION) * (2.0f * 3.1415926f * WHEEL_RADIUS);
                        float dist_r = ((float)chassis_enc_right / ENCODER_RESOLUTION) * (2.0f * 3.1415926f * WHEEL_RADIUS);

                        // (b) 根據差速公式：角度變化量(弧度) = (右輪距離 - 左輪距離) / 輪距
                        float delta_theta_rad = (dist_r - dist_l) / WHEEL_TRACK;

                        // (c) 轉換成度數並累加到全域角度變數中
                        chassis_angle_deg += (delta_theta_rad * 180.0f / 3.1415926f);

                        // ========================================================
        // 4. 呼叫 PID 大腦，並把結果交給馬達硬體 (剛剛漏掉的關鍵)
                // ==========================================

                // --- 左輪控制 ---
        pwm_cmd_left = PID_Calc(&pid_left, rpm_left_filter);

                if (pwm_cmd_left >= 0) {
                    // 正轉：IN1 高, IN2 低
                    HAL_GPIO_WritePin(GPIOC, CHASSIS_L_IN1_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, CHASSIS_L_IN2_Pin, GPIO_PIN_SET);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwm_cmd_left);
                } else {
                    // 反轉：IN1 低, IN2 高 (PWM 值必須轉回正數)
                    HAL_GPIO_WritePin(GPIOC, CHASSIS_L_IN1_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, CHASSIS_L_IN2_Pin, GPIO_PIN_RESET);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(-pwm_cmd_left));
                }

                // --- 右輪控制 ---
               pwm_cmd_right = PID_Calc(&pid_right, rpm_right_filter);

                if (pwm_cmd_right >= 0) {
                    // 正轉
                    HAL_GPIO_WritePin(GPIOC, CHASSIS_R_IN1_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOC, CHASSIS_R_IN2_Pin, GPIO_PIN_RESET);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)pwm_cmd_right);
                } else {
                    // 反轉
                    HAL_GPIO_WritePin(GPIOC, CHASSIS_R_IN1_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, CHASSIS_R_IN2_Pin, GPIO_PIN_SET);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(-pwm_cmd_right));
                }
    }
}
// ========================================================
// 基礎底盤運動控制函式 (必須加在 Chassis_TurnPrecise 上方)
// ========================================================
/**
  * @brief 原地左轉 (逆時針自轉)
  */
uint8_t chassis_motion_state = 0; // 0=停止或轉向中, 1=直線行駛中
float locked_heading_deg = 0.0f;  // 用來記住起步瞬間的那條直線角度

void Chassis_SpinLeft(float turn_rpm)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // [Sean 2026-09-20 debug] 確認邏輯有沒有被呼叫
    chassis_motion_state = 0; // 告訴系統現在不是直走
    pid_left.target_rpm  =  turn_rpm;
    pid_right.target_rpm = -turn_rpm;
    pid_left.integral = 0.0f; // 轉向時順便清空 PID 誤差積分
    pid_right.integral = 0.0f;
}

void Chassis_SpinRight(float turn_rpm)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // [Sean 2026-09-20 debug] 確認邏輯有沒有被呼叫
    chassis_motion_state = 0; // 告訴系統現在不是直走
    pid_left.target_rpm  = -turn_rpm;
    pid_right.target_rpm =  turn_rpm;
    pid_left.integral = 0.0f; // 轉向時順便清空 PID 誤差積分
    pid_right.integral = 0.0f;
}

// [Sean 2026-09-21] 連續P控制專用：只設定目標轉速，不清空積分。
// 跟Chassis_SpinLeft/SpinRight最大的差異在這裡——舊的離散版本每次呼叫都清積分，
// 連續控制每個vision frame都會呼叫，若清積分等於積分永遠來不及累積、克服不了靜摩擦力。
void Chassis_SteerP(float left_rpm, float right_rpm)
{
    chassis_motion_state = 0;
    pid_left.target_rpm  = left_rpm;
    pid_right.target_rpm = right_rpm;
}

void Chassis_Stop(void)
{
    chassis_motion_state = 0; // 告訴系統現在不是直走
    pid_left.target_rpm  = 0.0f;
    pid_right.target_rpm = 0.0f;

    // 💡 關鍵：煞車時務必「徹底歸零」歷史積分與微分！
    // 這樣下一次起步時，左右兩顆馬達才不會帶著原地的力量不對稱暴衝
    pid_left.integral = 0.0f;
    pid_right.integral = 0.0f;
    pid_left.error_last = 0.0f;
    pid_right.error_last = 0.0f;
}

// 🚀 新增：記錄當前斜坡轉速的靜態變數
float current_ramp_rpm = 0.0f;

void Chassis_Forward(float forward_rpm)
{
    // 剛從靜止轉為直走的第一瞬間
    if (chassis_motion_state == 0)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // [Sean 2026-09-20 debug] 確認Forward邏輯有沒有被呼叫
        pid_left.integral = 0.0f;
        pid_right.integral = 0.0f;
        pid_left.error_last = 0.0f;
        pid_right.error_last = 0.0f;
        locked_heading_deg = chassis_angle_deg;
        chassis_motion_state = 1;

        // 🚀 關鍵 1：不要從 0 開始跳變，從安全的低速 15 RPM 柔和起跑！
        current_ramp_rpm = 15.0f;
    }

    // 🚀 關鍵 2：斜坡加速 (Soft Start)
    // 每次收到前進指令時，如果還沒到目標速度，就以每幀 8 RPM 平滑爬升！
    // 這能把原本 0 -> 70 RPM 的「瞬間暴力跳變」，化為 200 毫秒的「舒適推背感加速」
    if (current_ramp_rpm < forward_rpm) {
        current_ramp_rpm += 8.0f;
        if (current_ramp_rpm > forward_rpm) current_ramp_rpm = forward_rpm;
    } else {
        current_ramp_rpm = forward_rpm;
    }

    // 電子羅盤糾偏
    float heading_error = locked_heading_deg - chassis_angle_deg;
    float comp = 0.0f;

    if (heading_error > 0.5f || heading_error < -0.5f)
    {
        comp = heading_error * 1.0f;
        if (comp > 8.0f)  comp = 8.0f;
        if (comp < -8.0f) comp = -8.0f;
    }

    // 將柔和爬升中的 current_ramp_rpm 餵給馬達
    pid_left.target_rpm  = current_ramp_rpm - comp;
    pid_right.target_rpm = current_ramp_rpm + comp;
}
/**
  * @brief 閉迴路精準轉向：轉動到指定的相對角度
  * @param target_angle_change 要轉動的角度 (正數為左轉/逆時針，負數為右轉/順時針)
  * @param spin_rpm            旋轉速度 (建議設定在 80~120 RPM)
  */
void Chassis_TurnPrecise(float target_angle_change, float spin_rpm)
{
    // 1. 記錄起轉前的初始角度，並計算最終目標角度
    float start_angle = chassis_angle_deg;
    float target_angle = start_angle + target_angle_change;

    // 2. 判斷轉向方向並啟動旋轉
    if (target_angle_change > 0) {
        Chassis_SpinLeft(spin_rpm);  // 角度增加：逆時針左轉
    } else {
        Chassis_SpinRight(spin_rpm); // 角度減少：順時針右轉
    }

    // 3. 迴圈等待，直到真實角度到達目標角度附近 (容許誤差 ±2 度)
    while (1)
    {
        float current_error = target_angle - chassis_angle_deg;

        // 如果誤差進入 ±2 度以內，代表轉到了！
        if (current_error <= 2.0f && current_error >= -2.0f) {
            break; // 跳出迴圈
        }

        // 短暫延遲避免卡死 CPU，讓背景 TIM6 中斷持續積分角度
        HAL_Delay(10);
    }

    // 4. 轉到位了！立刻煞車停止
    Chassis_Stop();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (command_ready == 0)
        {
            // 🚀 關鍵防護：如果緩衝區是空的 (rx_index == 0)，嚴格檢查第一個字元！
            // 只允許合法指令的開頭字母 (B=BALL, N=NOBALL, L=LED, P=PING, S=STOP) 進入
            // 這樣能徹底防止收進上一句殘留的半截數字 (如 "35.0,0.0\n")
        	if (rx_index == 0 && !(rx_byte == 'B' || rx_byte == 'N' || rx_byte == 'L' || rx_byte == 'P' || rx_byte == 'S' || rx_byte == 'C'))
            {
                // 如果不是合法指令開頭，直接丟棄，等待下一個字元
                HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
                return;
            }

            if (rx_index < (sizeof(rx_buffer) - 1))
            {
                rx_buffer[rx_index++] = rx_byte;

                if (rx_byte == '\n')
                {
                    rx_buffer[rx_index] = '\0';
                    command_ready = 1;
                }
            }
            else
            {
                rx_index = 0; // 防溢位重置
            }
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
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
