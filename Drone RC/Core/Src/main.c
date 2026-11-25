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
#include "NRF.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define pitch_min 460
#define pitch_max 3734
#define pitch_mid 2090
#define roll_min 443
#define roll_max 3733
#define roll_mid 2110
#define throttle_min 484
#define throttle_max 3579
#define throttle_mid 2070
#define yaw_min 400
#define yaw_max 3630
#define yaw_mid 2030
#define gimbal_deadband 40
#define pitch_mid_reversed (pitch_min + pitch_max - pitch_mid)
#define yaw_mid_reversed   (yaw_min + yaw_max - yaw_mid)
#define SBUS_MIN 192
#define SBUS_MID 992
#define SBUS_MAX 1794

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Radio configuration */
uint8_t TxAddress[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};

/* Gimbal readings and processed controls.
 * Channel order: pitch, roll, throttle, yaw. */
volatile uint16_t arr[4] = {0};  // Raw ADC readings updated by DMA.
uint16_t control_val[4] = {0};   // Calibrated readings.
uint16_t sbus_controls[4] = {0}; // Controls converted to SBUS values.

/* ADC monitoring: shared between callbacks and the main loop. */
volatile uint32_t last_adc_time = 0;
volatile uint8_t adc_ready = 0;
volatile uint8_t adc_fault = 0;

/* Arm switch state */
uint8_t arm = 0;

/* Radio packet */
uint8_t data[11] = {0};
uint16_t sequence = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
/* Check raw gimbal readings against plausible ADC limits. */
uint8_t gimbals_valid(const uint16_t raw[]);

/* Clamp readings, apply deadbands, and reverse pitch and yaw. */
void gimbal_calibration(const volatile uint16_t arr[], uint16_t controls[]);

/* Map a centred axis to the SBUS range using its calibrated endpoints and centre. */
static uint16_t map_centred_axis(int32_t value, int32_t minimum, int32_t centre, int32_t maximum);

/* Convert calibrated pitch, roll, throttle and yaw values to SBUS values. */
void controls_to_sbus(const uint16_t controls[], uint16_t sbus[]);

/* Read and debounce the arm switch: 0 = OFF, 1 = ON, 2 = invalid or unstable. */
uint8_t read_arm_switch(void);

/* Pack four controls, arm state and sequence number into an 11-byte radio packet. */
void pack_data(uint8_t data[], uint16_t sbus_controls[], uint8_t arm, uint16_t sequence);
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

    /* Initialize the HAL and system clock. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize peripherals. */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_ADC1_Init();
    MX_IWDG_Init();

    /* USER CODE BEGIN 2 */

    /* Deselect the radio before sending configuration commands. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* Configure the radio as a transmitter on channel 76. */
    NRF24_Init();
    NRF24_TxMode(TxAddress, 76);
    HAL_Delay(100);

    /* Start continuous ADC sampling into the four-channel DMA buffer. */
    last_adc_time = HAL_GetTick();

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)arr, 4) != HAL_OK)
    {
        adc_fault = 1;
    }

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        /* Read the callback timestamp first so it cannot be newer than now. */
        uint32_t adc_time = last_adc_time;
        uint32_t now = HAL_GetTick();

        /* Stop transmitting if ADC updates have been absent for 100 ms.
         * ADC faults remain latched until the transmitter restarts.
         */
        if ((uint32_t)(now - adc_time) >= 100U)
        {
            adc_fault = 1;
        }

        arm = read_arm_switch();

        /* Require ADC data, no ADC fault, and a stable, valid arm switch. */
        if (adc_ready && !adc_fault && arm != 2)
        {
            uint16_t raw[4];

            /* Copy readings so validation and calibration use the same values. */
            for (uint8_t i = 0; i < 4; i++)
            {
                raw[i] = arr[i];
            }

            /* Validate before clamping, which could hide an abnormal reading. */
            if (!gimbals_valid(raw))
            {
                adc_fault = 1;
            }
            else
            {
                /* Calibrate the gimbals and convert to SBUS control values. */
                gimbal_calibration(raw, control_val);
                controls_to_sbus(control_val, sbus_controls);

                /* Assign a new sequence number to each transmission attempt. */
                pack_data(data, sbus_controls, arm, sequence);
                sequence++;

                /* Toggle the indicator when transmission succeeds. */
                if (NRF24_Transmit(data) == 1)
                {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                }
            }
        }

        /* Refresh after completing the loop, even if transmission is disabled. */
        HAL_IWDG_Refresh(&hiwdg);

        /* Pause before processing the next set of controls. */
        HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Reload = 1999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void gimbal_calibration(const volatile uint16_t arr[], uint16_t controls[])
{
    for (int i = 0; i < 4; i++)
        controls[i] = arr[i];

    // Clamp values.
    if (controls[0] < pitch_min) controls[0] = pitch_min;
    else if (controls[0] > pitch_max) controls[0] = pitch_max;

    if (controls[1] < roll_min) controls[1] = roll_min;
    else if (controls[1] > roll_max) controls[1] = roll_max;

    if (controls[2] < throttle_min) controls[2] = throttle_min;
    else if (controls[2] > throttle_max) controls[2] = throttle_max;

    if (controls[3] < yaw_min) controls[3] = yaw_min;
    else if (controls[3] > yaw_max) controls[3] = yaw_max;

    // Centre deadbands.
    if (controls[0] >= pitch_mid - gimbal_deadband &&
        controls[0] <= pitch_mid + gimbal_deadband)
        controls[0] = pitch_mid;

    if (controls[1] >= roll_mid - gimbal_deadband &&
        controls[1] <= roll_mid + gimbal_deadband)
        controls[1] = roll_mid;

    if (controls[3] >= yaw_mid - gimbal_deadband &&
        controls[3] <= yaw_mid + gimbal_deadband)
        controls[3] = yaw_mid;

    // Throttle deadband near minimum.
    if (controls[2] <= throttle_min + gimbal_deadband)
        controls[2] = throttle_min;

    // Reverse yaw and pitch.
    controls[0] = pitch_max - (controls[0] - pitch_min);
    controls[3] = yaw_max - (controls[3] - yaw_min);
}

uint8_t read_arm_switch(void)
{
    static uint8_t previous = 2;
    static uint32_t changed_at = 0;

    GPIO_PinState pa10 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
    GPIO_PinState pa11 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);

    uint8_t position = 2;
    uint32_t now = HAL_GetTick();

    if (pa10 == GPIO_PIN_SET && pa11 == GPIO_PIN_RESET)
        position = 0;  // OFF
    else if (pa10 == GPIO_PIN_RESET && pa11 == GPIO_PIN_SET)
        position = 1;  // ON

    if (position != previous)
    {
        previous = position;
        changed_at = now;
    }

    if (position == 2 || (uint32_t)(now - changed_at) < 20U)
    {
        return 2;  // Invalid or not yet stable.
    }

    return position;
}

static uint16_t map_centred_axis(int32_t value, int32_t minimum, int32_t centre, int32_t maximum)
{
    if (value <= centre)
    {
        return SBUS_MIN
             + (value - minimum) * (SBUS_MID - SBUS_MIN)
             / (centre - minimum);
    }

    return SBUS_MID
         + (value - centre) * (SBUS_MAX - SBUS_MID)
         / (maximum - centre);
}

void controls_to_sbus(const uint16_t controls[], uint16_t sbus[])
{
    // Pitch: centre after reversal.
    sbus[0] = map_centred_axis(
        controls[0],
        pitch_min,
        pitch_min + pitch_max - pitch_mid,
        pitch_max
    );

    // Roll.
    sbus[1] = map_centred_axis(
        controls[1], roll_min, roll_mid, roll_max
    );

    // Throttle: map the full range without a centre.
    sbus[2] = SBUS_MIN
            + ((int32_t)controls[2] - throttle_min)
            * (SBUS_MAX - SBUS_MIN)
            / (throttle_max - throttle_min);

    // Yaw: centre after reversal.
    sbus[3] = map_centred_axis(
        controls[3],
        yaw_min,
        yaw_min + yaw_max - yaw_mid,
        yaw_max
    );
}

void pack_data(uint8_t data[], uint16_t sbus_controls[], uint8_t arm, uint16_t sequence){
	data[0] = sbus_controls[0] & 0xFF;
	data[1] = sbus_controls[0] >> 8;
	data[2] = sbus_controls[1] & 0xFF;
	data[3] = sbus_controls[1] >> 8;
	data[4] = sbus_controls[2] & 0xFF;
	data[5] = sbus_controls[2] >> 8;
	data[6] = sbus_controls[3] & 0xFF;
	data[7] = sbus_controls[3] >> 8;
	data[8] = arm;
	data[9] = sequence & 0xFF;
	data[10] = sequence >> 8;
	
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        last_adc_time = HAL_GetTick();
        adc_ready = 1;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_fault = 1;
    }
}

uint8_t gimbals_valid(const uint16_t raw[])
{
    // Initial tolerance beyond the measured stick travel.
    const int32_t margin = 150;

    return
        raw[0] >= pitch_min - margin &&
        raw[0] <= pitch_max + margin &&

        raw[1] >= roll_min - margin &&
        raw[1] <= roll_max + margin &&

        raw[2] >= throttle_min - margin &&
        raw[2] <= throttle_max + margin &&

        raw[3] >= yaw_min - margin &&
        raw[3] <= yaw_max + margin;
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
