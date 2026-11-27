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
#define LINK_TIMEOUT_MS 100U
#define SBUS_TX_TIMEOUT_MS 100U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t RxAddress[5] = {0xEE,0xDD,0xCC,0xBB,0xAA};
uint8_t st = 0;
uint8_t RxData[11];
uint16_t controls_local[4] = {0,0,0,0};
uint8_t arm_local = 0;
uint16_t controls[4] = {0,0,0,0};
uint8_t arm = 0;
uint16_t sequence = 0;
uint16_t old_sequence = 0;
uint8_t have_sequence = 0;
uint32_t last_valid_packet_time = 0;
uint8_t link_ok = 0;
uint8_t rx_flush_pending = 0;
uint16_t channels[16];
uint8_t sbus_frame[25];
uint32_t last_sbus_time = 0;
uint8_t arm_history[20];
uint16_t throttle_history[20];
uint8_t history_count = 0;
uint32_t last_sbus_success_time = 0;

volatile uint32_t sbus_tx_ok = 0;
volatile uint32_t sbus_tx_errors = 0;
volatile HAL_StatusTypeDef sbus_tx_status = HAL_OK;
volatile uint32_t rx_accepted = 0;
volatile uint32_t rx_invalid = 0;
volatile uint32_t rx_old_or_duplicate = 0;
volatile uint32_t rx_spi_errors = 0;
volatile uint32_t link_loss_count = 0;

uint8_t rf;
uint8_t cfg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void unpack_data(uint8_t RxData[], uint16_t controls[], uint8_t *arm, uint16_t *sequence);
uint8_t validate(uint16_t controls[], uint8_t arm);
uint8_t check_sequence(uint16_t sequence, uint16_t *old_sequence);
uint8_t arming(uint8_t arm_request, uint16_t throttle, uint8_t reset);
void pack_sbus(const uint16_t controls[], uint8_t arm, uint8_t frame[]);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	NRF24_Init();
	NRF24_RxMode(RxAddress, 76);
	
	HAL_Delay(100);  // Allow power-up to settle

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// Detect loss of valid, fresh radio packets.
    if (link_ok &&
        (uint32_t)(HAL_GetTick() - last_valid_packet_time) >= LINK_TIMEOUT_MS)
    {
				link_loss_count++;
        link_ok = 0;
        have_sequence = 0;
        arm = arming(0, 0, 1);  // Require OFF -> ON again.
				rx_flush_pending = 1;
    }
		
		if (rx_flush_pending)
		{
				if (NRF24_FlushRxChecked())
				{
						rx_flush_pending = 0;
				}
				else
				{
						rx_spi_errors++;
				}
		}

    // Process at most 3 packets per main-loop iteration.
    for (uint8_t count = 0; count < 3 && !rx_flush_pending; count++)
		{
				uint8_t fifo_status;

				if (nrf24_ReadRegChecked(FIFO_STATUS, &fifo_status) == 0)
				{
						rx_spi_errors++;
						break;
				}

				if (fifo_status & (1U << 0))
						break;

				if (NRF24_Receive(RxData) == 0)
				{
						rx_spi_errors++;
						break;
				}

				unpack_data(RxData, controls_local, &arm_local, &sequence);

				if (validate(controls_local, arm_local) == 0)
				{
						rx_invalid++;
						continue;
				}

				if (have_sequence == 0)
				{
						old_sequence = sequence;
						have_sequence = 1;
				}
				else if (check_sequence(sequence, &old_sequence) == 0)
				{
						rx_old_or_duplicate++;
						continue;
				}

				rx_accepted++;

				last_valid_packet_time = HAL_GetTick();
				link_ok = 1;

				for (uint8_t i = 0; i < 4; i++)
				{
						controls[i] = controls_local[i];
				}

				arm = arming(arm_local, controls_local[2], 0);
		}

    uint32_t now = HAL_GetTick();

    // Recheck freshness after processing, before sending SBUS.
    if (link_ok &&
        (uint32_t)(now - last_valid_packet_time) >= LINK_TIMEOUT_MS)
    {
        link_ok = 0;
        have_sequence = 0;
        arm = arming(0, 0, 1);
				rx_flush_pending = 1;
    }

    // Send SBUS every 10 ms while the radio link is valid.
		if (link_ok && (uint32_t)(now - last_sbus_time) >= 10U)
		{
				last_sbus_time = now;

				// Check BEFORE packing, so recovery cannot send ARM ON
				// after a prolonged gap in successful transmissions.
				if ((uint32_t)(HAL_GetTick() - last_sbus_success_time)
						>= SBUS_TX_TIMEOUT_MS)
				{
						arm = arming(0, 0, 1);
				}

				pack_sbus(controls, arm, sbus_frame);

				sbus_tx_status =
						HAL_UART_Transmit(&huart1, sbus_frame, 25, 5);

				if (sbus_tx_status == HAL_OK)
				{
						sbus_tx_ok++;
						last_sbus_success_time = HAL_GetTick();
				}
				else
				{
						sbus_tx_errors++;
				}
		}
				
    // Refresh after completing the main-loop work,
    // including when the RC is disconnected.
    HAL_IWDG_Refresh(&hiwdg);

    HAL_Delay(1);
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
  hiwdg.Init.Reload = 2499;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void unpack_data(uint8_t RxData[], uint16_t controls[], uint8_t *arm, uint16_t *sequence){
	controls[0] = RxData[0] | (RxData[1] << 8);
	controls[1] = RxData[2] | (RxData[3] << 8);
	controls[2] = RxData[4] | (RxData[5] << 8);
	controls[3] = RxData[6] | (RxData[7] << 8);
	*arm = RxData[8];
	*sequence = RxData[9] | (RxData[10] << 8);
}

uint8_t validate(uint16_t controls[], uint8_t arm){
	if (172 <= controls[0] && controls[0] <= 1811 &&
			172 <= controls[1] && controls[1] <= 1811 &&
			172 <= controls[2] && controls[2] <= 1811 &&
			172 <= controls[3] && controls[3] <= 1811 &&
			(arm == 0 || arm == 1)){
				return 1;
			}
			return 0;
}

uint8_t check_sequence(uint16_t sequence, uint16_t *old_sequence)
{
    uint16_t difference = (uint16_t)(sequence - *old_sequence);

    if (difference > 0 && difference < 32768U)
    {
        *old_sequence = sequence;
        return 1;
    }

    return 0;
}

uint8_t arming(uint8_t arm_request, uint16_t throttle, uint8_t reset)
{
    enum {
        WAIT_FOR_OFF,
        READY,
        ARM_ALLOWED
    };

    static uint8_t state = WAIT_FOR_OFF;

    if (reset)
    {
        state = WAIT_FOR_OFF;
        return 0;
    }

    switch (state)
    {
        case WAIT_FOR_OFF:
            if (arm_request == 0 && throttle < 200)
                state = READY;
            return 0;

        case READY:
            if (arm_request == 1)
            {
                if (throttle < 200)
                {
                    state = ARM_ALLOWED;
                    return 1;
                }

                // ON at high throttle: require OFF again.
                state = WAIT_FOR_OFF;
            }
            return 0;

        case ARM_ALLOWED:
            if (arm_request == 0)
            {
                state = (throttle < 200) ? READY : WAIT_FOR_OFF;
                return 0;
            }

            // Throttle may rise after arming is allowed.
            return 1;

        default:
            state = WAIT_FOR_OFF;
            return 0;
    }
}

void pack_sbus(const uint16_t controls[], uint8_t arm, uint8_t frame[])
{
    for (int i = 0; i < 25; i++)
        frame[i] = 0;

    frame[0] = 0x0F;

    for (int ch = 0; ch < 16; ch++)
    {
        uint16_t value;

        switch (ch)
        {
            case 0: value = controls[1]; break;  // Roll
            case 1: value = controls[0]; break;  // Pitch
            case 2: value = controls[2]; break;  // Throttle
            case 3: value = controls[3]; break;  // Yaw
            case 4: value = arm ? 1811 : 172; break;
            default: value = 172; break;
        }

        for (int b = 0; b < 11; b++)
        {
            if (value & (1U << b))
            {
                int bit = ch * 11 + b;

                frame[1 + bit / 8] |=
                    (uint8_t)(1U << (bit % 8));
            }
        }
    }

    frame[23] = 0x00;  // Flags
    frame[24] = 0x00;  // End byte
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
