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
#include "uc1701.h"
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
#define FACTOR_OVP 86 // map(0.3V, 0, 11.145, 0, 4027);
// #define GAIN_OCP 21.5 // Gain of output current sense
#define ADC_MAX 4027 // maximum value of adc at 3.327V is 4027
#define IOUT_ZERO 1200 // At 0A --> adc value is 1200
#define IOUT_MAX 23 // Iout_max = 23A
#define VOUT_MAX 10.75 // Vout max = 10.7V
#define UVLO_MAX 26.322 // UVLO_max = 26.322V

#define DAC_SETTING (uint16_t)0x7000u // bit15=0: channel A; bit14=1: buffered; bit13=1: gain x1; bit12=1: output enable

#define PG_TH 0.9 // 90% VOUT
#define PG_HYSTERESIS 0.05 // 5% VOUT

#define STARTPAGE ((uint32_t)0x08007C00) // Page 31 - Sector 7

#define LCD_DELAY250 250// LCD refresh after each 250ms
#define MODE_COMMAND GPIO_PIN_RESET
#define MODE_DATA 	 GPIO_PIN_SET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
extern const uint8_t uc1701_font_UVLO[];
extern const uint8_t uc1701_font_SCP[];
extern const uint8_t uc1701_font_OVP[];
extern const uint8_t uc1701_font_OCP[];

UC1701_HandleTypedef UC1701;
FLASH_EraseInitTypeDef erase_init;
uint32_t erase_error_check = 0;

uint32_t adc_value[3]; // ADC realtime values: [0]A_UVLO, [1]A_VOUT, [2]A_CSENSE in range 0~4027
uint8_t uart_rx_value[10]; // UART_RX buffer
uint16_t vref = 0; // DACVREF in range 0~4027
uint32_t uvlo_compare = 0; // UVLO comparing value in range 0~4027
uint32_t ovp_compare = 0; // OVP comparing value in range 0~4027
uint32_t ocp_compare = 0; // OCP comparing value in range 0~4027
uint32_t pw_good_th = 0; // PGood threshold value in range 0~4027
uint32_t pw_good_hys = 0; // PGood hysteresis value in range 0~4027

float uvlo_flash = 0; // UVLO flash value in (V). UVLO_max = 26.322V
float vout_flash = 0; // VOUT flash value in (V). VOUT_max = 10.8V
float ocp_flash = 0; // OCP flash value in (A). OCP_max = 23A

float temp_val = 0; // temporary value

uint32_t current_warning = WARNING_NONE;
uint32_t current_mode = MODE_NONE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void UC1701_config(void);
static void Buzz_bip_warning(void);
static void Buzz_bip_button(void);
static float map(float value, float fromLow, float fromHigh, uint32_t toLow, uint32_t toHigh);

void Setup_values(); // Converting user values to 12-bit values
void Flash_read_write(uint32_t flash_OP); // Read or write user values to flash memory
void Reset_current_state(); // Reset current state
void DAC_send(uint16_t *vref); // Send command to DACVREF
//
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
	uint32_t adc_value[3]; // ADC realtime values: [0]A_UVLO, [1]A_VOUT, [2]A_CSENSE in range 0~4027

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  UC1701_config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM14_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc, adc_value, 3);
	uc1701Init(&UC1701); // LCD init, setup LCD and clear screen
	
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = STARTPAGE;
	erase_init.NbPages = 1;
	
  /** @brief Initial values:
   * UVLO at 11V
   * VOUT at 3.3V
   * Overvoltage protection at VOUT + 0.3V
   * LED Power good threshold (Vth(+) = 3.135V)
   * LED Power good hysteresis (Vth(-) = 2.805V)
   * Overcurrent protection at 10A
   * @note Initial values are stored in flash memory
  */
  Flash_read_write(FLASH_READ);
	
  BEGIN_SWITCH_ON: // Input voltage from source come into Power circuitry
  HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_SET); // Switch ON
	HAL_Delay(1000);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); // Start timer, output PWM 50kHz 2.5%
  // Check Short circuit at the beginning
  if((HAL_GPIO_ReadPin(S_SCP_GPIO_Port, S_SCP_Pin) \
    /* & HAL_GPIO_ReadPin(C_SCP_GPIO_Port, C_SCP_Pin) */) == 1)
  {
    HAL_Delay(100); // delay 100ms
  }
  else{
    current_warning = WARNING_SCP;
    HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
    // Display LCD: SCP
    uc1701Write_warning((uint8_t*)uc1701_font_SCP, FONT_SCP_LENGTH);
    while(current_warning != WARNING_NONE)
    { // wait for OK button
      Reset_current_state();
      Buzz_bip_warning();
    }
    uc1701Fill(0); // clear LCD after warning
    goto BEGIN_SWITCH_ON;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  BEGIN_OUTPUT: // Exporting VOUT and measure ADC values
  HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET);
  uint32_t LCD_tickstart = HAL_GetTick(); // LCD timepoint to refresh
  uc1701Write_skeleton();
  uc1701Write_setup_value(uvlo_flash, vout_flash, ocp_flash);
  // While infinity loop
  while (1)
  {
    // Shortcircuit protection
    if(current_warning == WARNING_SCP)
    {
      // Display LCD: SCP
      uc1701Fill(0); // clear LCD before warning
      uc1701Write_warning((uint8_t*)uc1701_font_SCP, FONT_SCP_LENGTH);
      while(current_warning != WARNING_NONE)
      { // wait for OK button
        Reset_current_state();
        Buzz_bip_warning();
      }
      uc1701Fill(0); // clear LCD after warning
      goto BEGIN_SWITCH_ON;
    }
    // Undervoltage lockout
    if(adc_value[0] <= uvlo_compare) // A_UVLO <= UVLO
    {
      current_warning = WARNING_UVLO;
      HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
      HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
      // Display LCD: UVLO
      uc1701Fill(0); // clear LCD before warning
      uc1701Write_warning((uint8_t*)uc1701_font_UVLO, FONT_UVLO_LENGTH);
      while (current_warning != WARNING_NONE)
      { // wait for OK button
        Reset_current_state();
        Buzz_bip_warning();
      }
      uc1701Fill(0); // clear LCD after warning
      goto BEGIN_SWITCH_ON;
    }
    // Overvoltage protection
    if(adc_value[1] >= ovp_compare) // A_VFB >= OVP
    {
      current_warning = WARNING_OVP;
      HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
      // Display LCD: OVP
      uc1701Fill(0); // clear LCD before warning
      uc1701Write_warning((uint8_t*)uc1701_font_OVP, FONT_OVP_LENGTH);
      while (current_warning != WARNING_NONE)
      { // wait for OK button
        Reset_current_state();
        Buzz_bip_warning();
      }
      uc1701Fill(0); // clear LCD after warning
      goto BEGIN_SWITCH_ON;
    }
    // Overcurrent protection
    if(adc_value[2] >= ocp_compare)
    { // A_CSENSE >= OCP
      current_warning = WARNING_OCP;
      HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
      // Display LCD: OCP
      uc1701Fill(0); // clear LCD before warning
      uc1701Write_warning((uint8_t*)uc1701_font_OCP, FONT_OCP_LENGTH);
      while (current_warning != WARNING_NONE)
      { // wait for OK button
        Reset_current_state();
        Buzz_bip_warning();
      }
      uc1701Fill(0); // clear LCD after warning
      goto BEGIN_OUTPUT;
    }
    // LED power good
    if(adc_value[1] >= (pw_good_th + pw_good_hys))
    {
      HAL_GPIO_WritePin(PGOOD_GPIO_Port, PGOOD_Pin, GPIO_PIN_RESET);
    }
    else if (adc_value[1] <= (pw_good_th - pw_good_hys))
    {
      HAL_GPIO_WritePin(PGOOD_GPIO_Port, PGOOD_Pin, GPIO_PIN_SET);
    }
		
    if((HAL_GetTick() - LCD_tickstart) >= LCD_DELAY250)
    {
      // -------------------------------------
      // Display Voltage, Current, Power change on LCD
      // -------------------------------------
      uc1701Write_ADC_value(adc_value);
			LCD_tickstart = HAL_GetTick();
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
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
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
  htim14.Init.Period = 960-1;
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
  sConfigOC.Pulse = 24;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : PA3 PA5 PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/** @brief Converting user values to 12-bit values and set new VOUT
 * @retval NONE
*/
void Setup_values()
{
  uint16_t vout = map(vout_flash, 0, VOUT_MAX, 0, 4095);
  uvlo_compare = map(uvlo_flash, 0, UVLO_MAX, 0, ADC_MAX);
  ovp_compare = vout + FACTOR_OVP;
  ocp_compare = map(ocp_flash, 0, IOUT_MAX, IOUT_ZERO, ADC_MAX);
  pw_good_th = vout * PG_TH;
  pw_good_hys = vout * PG_HYSTERESIS;
  // Send vout to output through DACVREF.
  DAC_send(&vout);
}

/** @brief Read or write user values to FLASH memory, then setup comparing values
 * @param flash_OP: types in FLASH_status typedef
 * @retval NONE
*/
void Flash_read_write(uint32_t flash_OP)
{
  uint32_t temp = 0;
  switch (flash_OP)
  {
    case FLASH_READ:
      // 1. Read data from flash memory
	    temp = *(uint32_t*)(STARTPAGE);
      uvlo_flash = *(float*)&temp;

      temp = *(uint32_t*)(STARTPAGE+4u);
      vout_flash = *(float*)&temp;

      temp = *(uint32_t*)(STARTPAGE+8u);
      ocp_flash = *(float*)&temp;
      // 3. Setup comparing values after changing
      Setup_values();
      break;
    
    case FLASH_WRITE:
      // 2. Write data from flash memory
      HAL_FLASH_Unlock();
      HAL_FLASHEx_Erase(&erase_init, &erase_error_check);

      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STARTPAGE, *(uint32_t*)&uvlo_flash);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STARTPAGE + 4u, *(uint32_t*)&vout_flash);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STARTPAGE + 8u, *(uint32_t*)&ocp_flash);

      HAL_FLASH_Lock();
      // 3. Setup comparing values after changing
      Setup_values();
      break;
    default:
      break;
  }
}
/** @brief Reset current mode and temporary value
 * @retval NONE
*/
void Reset_current_state()
{
  // Reset all mode setup value
  current_mode = MODE_NONE;
  temp_val = 0;
}

/** @brief Send command to DACVREF
 * @param vref: pointer of value in range 0~4095
 * @retval NONE
*/
void DAC_send(uint16_t *vref)
{
  *vref |= DAC_SETTING;
  uint8_t temp[] = {(*vref >> 8u), (*vref & 0x00FFu)};
  HAL_GPIO_WritePin(CSDAC_GPIO_Port, CSDAC_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, temp, 2, 100);
  HAL_GPIO_WritePin(CSDAC_GPIO_Port, CSDAC_Pin, GPIO_PIN_SET);
}





/** @brief Function processing button interrupt
 * @param GPIO_pin: GPIO pin name
 * @retval NONE
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case S_SCP_Pin:
      current_warning = WARNING_SCP;
      HAL_GPIO_WritePin(C_SCP_GPIO_Port, C_SCP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
      HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
      break;
    case BT_MODE_Pin:
      Buzz_bip_button();
      switch (current_mode)
      {
        case MODE_NONE:
          temp_val = uvlo_flash;
          // Set UVLO mode cursor
          if(current_warning == WARNING_NONE)
          {
            uc1701SetPosition(92, 0);
            uc1701Write_string("*",1);
          }
          current_mode++;
          break;
        case MODE_SET_UVLO:
          temp_val = vout_flash;
          // Erase UVLO mode cursor
          if(current_warning == WARNING_NONE)
          {
            uc1701SetPosition(92, 0);
            uc1701Write_string(" ",1);
            // Set VOUT mode cursor
            uc1701SetPosition(92, 3);
            uc1701Write_string("*",1);
          }
          current_mode++;
          break;
        case MODE_SET_VOUT:
          temp_val = ocp_flash;
          // Erase VOUT mode cursor
          if(current_warning == WARNING_NONE)
          {
            uc1701SetPosition(92, 3);
            uc1701Write_string(" ",1);
            // Set OCP mode cursor
            uc1701SetPosition(98, 6);
            uc1701Write_string("*",1);
          }
          current_mode++;
          break;
        case MODE_SET_OCP:
          temp_val = 0;
          // Erase OCP mode cursor
          if(current_warning == WARNING_NONE)
          {
            uc1701SetPosition(98, 6);
            uc1701Write_string(" ",1);
          }
          current_mode = MODE_NONE;
          break;
        default:
          break;
      }
      break;
    case BT_PLUS_Pin:
      Buzz_bip_button();
      temp_val += 0.1;
      switch (current_mode)
      {
        case MODE_NONE:
          // do nothing
          break;
        case MODE_SET_UVLO:
          uc1701Write_setup_value(temp_val, vout_flash, ocp_flash);
          break;
        case MODE_SET_VOUT:
          uc1701Write_setup_value(uvlo_flash, temp_val, ocp_flash);
          break;
        case MODE_SET_OCP:
          uc1701Write_setup_value(uvlo_flash, vout_flash, temp_val);
          break;
        default:
          break;
      }
      break;
    case BT_MINUS_Pin:
      Buzz_bip_button();
      temp_val -= 0.1;
      switch (current_mode)
      {
        case MODE_NONE:
          // do nothing
          break;
        case MODE_SET_UVLO:
          uc1701Write_setup_value(temp_val, vout_flash, ocp_flash);
          break;
        case MODE_SET_VOUT:
          uc1701Write_setup_value(uvlo_flash, temp_val, ocp_flash);
          break;
        case MODE_SET_OCP:
          uc1701Write_setup_value(uvlo_flash, vout_flash, temp_val);
          break;
        default:
          break;
      }
      break;
    case BT_OK_Pin:
      Buzz_bip_button();
      // 1. Reset warning
      if(current_warning != WARNING_NONE)
      {
        current_warning = WARNING_NONE;
      }
      // 2. Save changes
      switch (current_mode)
      {
        case MODE_NONE:
          // do nothing
          break;
        case MODE_SET_UVLO:
          uvlo_flash = temp_val;
          Flash_read_write(FLASH_WRITE);
          uc1701SetPosition(92, 0);
          uc1701Write_string(" ",1);
          uc1701Write_setup_value(uvlo_flash, vout_flash, ocp_flash);
          current_mode = MODE_NONE;
          break;
        case MODE_SET_VOUT:
          vout_flash = temp_val;
          Flash_read_write(FLASH_WRITE);
          uc1701SetPosition(92, 3);
          uc1701Write_string(" ",1);
          uc1701Write_setup_value(uvlo_flash, vout_flash, ocp_flash);
          current_mode = MODE_NONE;
          break;
        case MODE_SET_OCP:
          ocp_flash = temp_val;
          Flash_read_write(FLASH_WRITE);
          uc1701SetPosition(98, 6);
          uc1701Write_string(" ",1);
          uc1701Write_setup_value(uvlo_flash, vout_flash, ocp_flash);
          current_mode = MODE_NONE;
          break;
        default:
          break;
      }
	}
}

/** @brief UC1701 config GPIO
 * 	@retval None.
 */
static void UC1701_config()
{
	UC1701.CS_port = CSLCD_GPIO_Port;
	UC1701.CS_pin = CSLCD_Pin;
	UC1701.CD_port = CD_GPIO_Port;
	UC1701.CD_pin = CD_Pin;
	UC1701.RESET_port = RESETLCD_GPIO_Port;
	UC1701.RESET_pin = RESETLCD_Pin;
	UC1701.p_spi = &hspi1;
}

/** @brief Buzzer bip warning 5 bips in 1 second
 * 	@retval None.
 */
static void Buzz_bip_warning(void)
{
  for(int i = 0; i<5; i++)
	{
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		HAL_Delay(20);
    HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
    HAL_Delay(180);
	}
	HAL_Delay(1000);
}

/** @brief Buzzer bip when button click
 * 	@retval None.
 */
static void Buzz_bip_button(void)
{
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
}

/** @brief Re-maps a number from one range to another.
 * 	@param value: the number to be converted.
 * 	@param fromLow: the lower bound of the values current range.
 *	@param fromHigh: the upper bound of the values current range.
 * 	@param toLow: the lower bound of the values target range.
 *	@param toHigh: the upper bound of the values target range.
 * 	@retval None.
 */
static float map(float value, float fromLow, float fromHigh, uint32_t toLow, uint32_t toHigh)
{
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
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
