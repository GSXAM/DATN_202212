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
#include<stdint.h>
//#include "stm32f0xx_hal_flash_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum WARNING_status
{
  WARNING_NONE = 0u,
  WARNING_OVP,
  WARNING_OCP,
  WARNING_UVLO,
  WARNING_SCP
}WARNING_status;

typedef enum MODE_status
{
  MODE_NONE = 0u,
  MODE_SET_UVLO,
  MODE_SET_VOUT,
  MODE_SET_OCP
}MODE_status;

typedef enum FLASH_status
{
  FLASH_READ = 0u,
  FLASH_WRITE
}FLASH_status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FACTOR_UVLO 156.844 // =(4095/3.3V)*(6.8k/(6.8k+47k))
#define FACTOR_VOUT 310.227 // =(4095/3.3V)*(15.67k/(15.67k+47k))
#define FACTOR_OVP 93.0681 // =0.3V*(4095/3.3V)*(15.67k/(15.67k+47k))
#define PG_TH 0.9 // 90% VOUT
#define PG_HYSTERESIS 0.05 // 5% VOUT
#define LCD_delay 250 // 100ms
#define STARTPAGE (uint32_t)0x08007C00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FACTOR_OCP(I_set) (1240.909 + I_set * 124.0909) // =(4095/3.3)*1 + I_set *((4095/3.3)/10A)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

FLASH_EraseInitTypeDef erase_init;
uint32_t erase_error_check = 0;

uint32_t adc_value[3]; // Position: [0]A_UVLO, [1]A_VFB, [2]A_CSENSE
uint8_t uart_rx_value[10];

float val_set[3]; // Position: [0]uvlo, [1]vout, [2]ocp in 0-4095
float ovp = 0; // Overvoltage in 0-4095

float pw_good_th = 0; // Power good threshold in 0-4095
float pw_good_hys = 0; // Power good hysteresis in 0-4095

float temp_val = 0; // Temporary value in (v) or (A)

uint32_t current_mode = MODE_NONE; // Biến chỉ trạng thái MODE_status
uint32_t current_warning = WARNING_NONE; // Biến chỉ trạng thái WARNING_status

// Các giá trị bên dưới dùng để đọc hoặc ghi từ bộ nhớ FLASH
float uvlo; // Giá trị cài đặt UVLO in (V)
float vout; // Giá trị cài đặt VOUT in (V)
float ocp;  // Giá trị cài đặt OCP in (A)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void Setup_value_compare(); // Cài đặt giá trị so sánh: UVLO, VOUT, OCP, OVP, POWER_GOOD threshold
void Flash_read_write(uint32_t flash_OP); // Đọc ghi dữ liệu vào FLASH
void Reset_current_state(); // Reset all mode setup value

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
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  // Khởi tạo ADC, DMA, SPI, UART, LCD.
	HAL_ADC_Start_DMA(&hadc, adc_value, sizeof(adc_value));
	HAL_UART_Receive_DMA(&huart1, uart_rx_value, sizeof(uart_rx_value)); // Note: dữ liệu mới tự động ghi đè khi tràn vùng nhớ
	
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = STARTPAGE;
	erase_init.NbPages = 1;
	
  /** @brief Giá trị khởi tạo từ bộ nhớ FLASH như sau:
   * val_set[3]: Position [0]uvlo, [1]vout, [2]ocp
   * Bảo vệ sụt áp tại 11V
   * Cài đặt điện áp ra 3.3V
   * Bảo vệ quá áp khi vượt quá 0.3V
   * Ngưỡng kích hoạt LED Power good (Vth(+) = 3.135V)
   * Khoảng trễ LED Power good (Vth(-) = 2.805V)
   * Bảo vệ quá dòng 10A
  */
  Flash_read_write(FLASH_READ);
	
  BEGIN_SWITCH_ON: // Bắt đầu đóng điện vào mạch công suất
  HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_SET); // Đóng điện vào
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); // Start timer, output PWM 50kHz 2.5%

  // Kiểm tra ngắn mạch ngay từ khi bắt đầu chạy
  if((HAL_GPIO_ReadPin(S_CSP_GPIO_Port, S_CSP_Pin) & HAL_GPIO_ReadPin(C_SCP_GPIO_Port, C_SCP_Pin)) == 1){
    HAL_Delay(100); // delay 100ms
  }
  else{
    current_warning = WARNING_SCP;
    HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
    // Hiển thị LCD ngắn mạch
    while (current_warning != WARNING_NONE){ // wait for OK button
      Reset_current_state();
    }
    goto BEGIN_SWITCH_ON;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  BEGIN_OUTPUT: // Bắt đầu xuất điện áp ra VOUT và đo đạc ADC
  HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET);
  uint32_t LCD_delay_start = HAL_GetTick(); // bắt đầu đếm thời gian chờ để cập nhật thông tin LCD
  while (1)
  {
    // Shortcircuit protection
    if(current_warning == WARNING_SCP){
      while(current_warning != WARNING_NONE){ // wait for OK button
        Reset_current_state();
      }
      goto BEGIN_SWITCH_ON;
    }
    // Undervoltage lockout
    if(adc_value[0] <= val_set[0]){ // A_UVLO <= UVLO
      current_warning = WARNING_UVLO;
      HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
      HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
      // Hiển thị LCD sụt áp
      //
      while (current_warning != WARNING_NONE){ // wait for OK button
        Reset_current_state();
      }
      goto BEGIN_SWITCH_ON;
    }
    // Overvoltage protection
    if(adc_value[1] >= ovp){ // A_VFB >= OVP
      current_warning = WARNING_OVP;
      HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
      // Hiển thị LCD quá áp
      //
      while (current_warning != WARNING_NONE){ // wait for OK button
        Reset_current_state();
      }
      goto BEGIN_SWITCH_ON;
    }
    // Overcurrent protection
    if(adc_value[2] >= val_set[2]){ // A_CSENSE >= OCP
      current_warning = WARNING_OCP;
      HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
      // Hiển thị LCD quá dòng
      //
      while (current_warning != WARNING_NONE){ // wait for OK button
        Reset_current_state();
      }
      goto BEGIN_OUTPUT;
    }
    // LED power good
    if(adc_value[1] >= (pw_good_th + pw_good_hys)){
      HAL_GPIO_WritePin(PGOOD_GPIO_Port, PGOOD_Pin, GPIO_PIN_RESET);
    }else if (adc_value[1] <= (pw_good_th - pw_good_hys)){
      HAL_GPIO_WritePin(PGOOD_GPIO_Port, PGOOD_Pin, GPIO_PIN_SET);
    }

    /**
     * Hiển thị công suất, dòng, áp lên LCD.
     * Chu kỳ thay đổi dữ liệu LCD là 250ms.
    */
    if((HAL_GetTick() - LCD_delay_start) >= LCD_delay){
      // Thay đổi thông số LCD
      //
      LCD_delay_start = HAL_GetTick(); // cập nhật thời gian
    }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 960-1; // 50kHz
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 24; // 2.5%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_Pin|CSLCD_Pin|PGOOD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, C_SCP_Pin|BUZZ_Pin|CD_Pin|RESETLCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSDAC_GPIO_Port, CSDAC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SD_Pin BUZZ_Pin PGOOD_Pin */
  GPIO_InitStruct.Pin = SD_Pin|BUZZ_Pin|PGOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : C_SCP_Pin */
  GPIO_InitStruct.Pin = C_SCP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(C_SCP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S_CSP_Pin BT_PLUS_Pin BT_OK_Pin */
  GPIO_InitStruct.Pin = S_CSP_Pin|BT_PLUS_Pin|BT_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_MODE_Pin BT_minus_Pin */
  GPIO_InitStruct.Pin = BT_MODE_Pin|BT_minus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CD_Pin RESETLCD_Pin CSLCD_Pin */
  GPIO_InitStruct.Pin = CD_Pin|RESETLCD_Pin|CSLCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CSDAC_Pin */
  GPIO_InitStruct.Pin = CSDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSDAC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0); // MODE, (-) button
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0); // S_SCP, OK, (+) button
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void Setup_value_compare()
{
  val_set[0] = uvlo * FACTOR_UVLO;
  val_set[1] = vout * FACTOR_VOUT;
  ovp = val_set[1] + FACTOR_OVP;
  pw_good_th = val_set[1]*PG_TH;
  pw_good_hys = val_set[1]*PG_HYSTERESIS;
  val_set[2] = FACTOR_OCP(ocp);
}

void Flash_read_write(uint32_t flash_OP)
{
  switch (flash_OP)
  {
    case FLASH_READ:
      // 1. Đọc lại dự liệu từ bộ nhớ flash
      uvlo = *(uint32_t*)(STARTPAGE);
      vout = *(uint32_t*)(STARTPAGE + 4u);
      ocp = *(uint32_t*)(STARTPAGE + 8u);
      // 3. Cài đặt giá trị so sánh
      Setup_value_compare();
      break;
    
    case FLASH_WRITE:
      // 2. Ghi dữ liệu vào bộ nhớ flash
      HAL_FLASH_Unlock();
      HAL_FLASHEx_Erase(&erase_init, &erase_error_check);

      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STARTPAGE, uvlo);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STARTPAGE + 4u, vout);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STARTPAGE + 8u, ocp);

      HAL_FLASH_Lock();
      // 3. Cài đặt giá trị so sánh
      Setup_value_compare();
      break;
    default:
      break;
  }
}

void Reset_current_state()
{
  // Reset all mode setup value
    current_mode = MODE_NONE;
    temp_val = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case S_CSP_Pin:
      current_warning = WARNING_SCP;
      HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
      HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
      // Hiển thị LCD ngắn mạch
      //
      break;
    case BT_MODE_Pin:
      switch (current_mode)
      {
        case MODE_NONE:
          temp_val = uvlo;
          current_mode++;
          break;
        case MODE_SET_UVLO:
          temp_val = vout;
          current_mode++;
          break;
        case MODE_SET_VOUT:
          temp_val = ocp;
          current_mode++;
          break;     
        case MODE_SET_OCP:
          temp_val = 0;
          current_mode = MODE_NONE;
          break;
        default:
          break;
      }
      break;
    case BT_PLUS_Pin:
      temp_val += 0.1;
      break;
    case BT_minus_Pin:
      temp_val -= 0.1;
      break;
    case BT_OK_Pin:
      switch (current_mode)
      {
        case MODE_NONE:
          //1. Reset warning
          if(current_warning != WARNING_NONE){
            current_warning = WARNING_NONE;
          }
          break;
        case MODE_SET_UVLO:
          uvlo = temp_val;
          Flash_read_write(FLASH_WRITE);
          break;
        case MODE_SET_VOUT:
          vout = temp_val;
          Flash_read_write(FLASH_WRITE);
          break;
        case MODE_SET_OCP:
          ocp = temp_val;
          Flash_read_write(FLASH_WRITE);
          break;
        default:
          break;
      }
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
