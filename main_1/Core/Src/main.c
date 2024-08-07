/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include "ltc2460.h"
#include "string.h"
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
typedef enum{
	VALVE,
	MAIN1,
	MAIN2,
	MAIN3,
	GROUND
} SEND_TO;

#define TX_BUFF_SIZE_MAIN3 10
#define TX_BUFF_SIZE_VALVE 1
#define RX_BUFF_SIZE_MAIN3 8
#define RX_BUFF_SIZE_VALVE 26
#define ADDRESS 2024   // inspire from this year

uint8_t rx_Buff_3[RX_BUFF_SIZE_MAIN3];   // receive from main3
uint8_t rx_Buff_V[RX_BUFF_SIZE_VALVE];   // receive from valve

uint8_t tx_Buff_3[TX_BUFF_SIZE_MAIN3];   // send to main3
uint8_t tx_Buff_V[TX_BUFF_SIZE_VALVE];   // send to valve

uint8_t rx_Buff_PC[1];   // receive from PC

int valid_valve_com = 0;
uint8_t command;
uint8_t Buff_size;
uint8_t count;   // adjustment of downlink rate
uint8_t ck_a_rx3, ck_b_rx3;   // for checksum
uint8_t ck_a_rxV, ck_b_rxV;   // for checksum
uint8_t ck_a_tx3, ck_b_tx3;   // for checksum
uint8_t ck_a_txV, ck_b_txV;   // for checksum
int flag;   // use as bool function. True:1, False:0
uint8_t PHASE;   // for phase
uint8_t TankPressure;   // for tank pressure
SEND_TO sendto, sendfrom, wantto;
uint8_t voltage_send;
int valve_received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
void send_MAIN3(uint8_t *tx_Buff);
void send_VALVE(uint8_t *tx_Buff, uint8_t com);
void calculateChecksum(uint8_t *Buff, uint8_t Buff_size, uint8_t *ck_a, uint8_t *ck_b);  // for checksum
int checkUART(uint8_t *rx_Buff, uint8_t *ck_a, uint8_t *ck_b);   // if True, it adjusts Buff_size, PHASE
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LTC2460_HandleTypeDef ltc2460;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
	ltc2460.speed = 0;
	ltc2460.spi = &hspi2;
	ltc2460.miso = GPIOB;
	ltc2460.ss = GPIOD;
	ltc2460.miso_num = GPIO_PIN_14;
	ltc2460.ss_num = GPIO_PIN_0;
	ltc2460.receive_only = 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SCB->VTOR = FLASH_BASE;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart1, rx_Buff_V, 1);

  HAL_UART_Receive_IT(&huart2, rx_Buff_3, RX_BUFF_SIZE_MAIN3);

  HAL_UART_Receive_IT(&huart4, rx_Buff_PC, 1);

  PHASE = 255;
  TankPressure = 255;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint16_t voltage;

    LTC2460_READ(&ltc2460, &voltage);
    /// 　計算上は109
    ///   実測で130
    int voltage_micro = voltage * 436 - 7138085;
    printf("tick: %d valid_valve_com: %d tank_pressure: %d phase: %d voltage: %d.%06d raw: %d\r\n",HAL_GetTick(),valid_valve_com, TankPressure,PHASE, voltage_micro/1000000, voltage_micro%1000000, voltage);
    voltage_send = voltage_micro / 100000;
	send_MAIN3(tx_Buff_3);
	printf("Voltage_send:%d\r\n", voltage_send);
	HAL_UART_Transmit(&huart2, tx_Buff_3, TX_BUFF_SIZE_MAIN3, 10);


    ///HAL_UART_Transmit(&huart2, tx_Buff_3, TX_BUFF_SIZE_MAIN3, 10);
    HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart4,(uint8_t *)ptr,len,10);
  return len;
}

void calculateChecksum(uint8_t *Buff, uint8_t Buff_size, uint8_t *ck_a, uint8_t *ck_b){
	*ck_a = 0;
    *ck_b = 0;

    for (int i = 0; i < (Buff_size - 2); i++) {
        *ck_a = (*ck_a + Buff[i]) & 0xFF;
        *ck_b = (*ck_b + *ck_a) & 0xFF;
    }
	printf("ck_a : %d\r\n", *ck_a);
	printf("ck_b : %d\r\n", *ck_b);
}


int checkUART(uint8_t *rx_Buff, uint8_t *ck_a, uint8_t *ck_b){
  int flag = 0;
	if (((rx_Buff[0] << 8) | (rx_Buff[1])) == ADDRESS || rx_Buff[2] == 2){
        Buff_size = rx_Buff[4];
		calculateChecksum(rx_Buff, Buff_size, ck_a, ck_b);

		if (*ck_a == rx_Buff[Buff_size-2] || *ck_b == rx_Buff[Buff_size-1]){
			flag = 1;
		}else{
			flag = 0;
		}

	}else{
		flag = 0;
	}
  return flag;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //Valve
	if (huart->Instance == USART1)
	{
    printf("Received from Valve\r\n");
    if(!valid_valve_com){
    	if(rx_Buff_V[0] == 0x24)
    	    {
    		HAL_UART_Receive_IT(&huart1, &rx_Buff_V[1], RX_BUFF_SIZE_VALVE - 1);
    		valid_valve_com = 1;
    		return;
    	    }else{
    	    	  MX_USART1_UART_Init();
				  memset(rx_Buff_V, 0, RX_BUFF_SIZE_VALVE);
				  HAL_UART_Receive_IT(&huart1, rx_Buff_V, 1);
				  return;
    	    }
    }else
    {
    	TankPressure = rx_Buff_V[1];
    	PHASE = rx_Buff_V[4];
    	valve_received = 1;
    	printf("TP:%d\r\nPHASE:%d\r\n", TankPressure, PHASE);
    }
      valid_valve_com = 0;
	  flag = 0;
	  MX_USART1_UART_Init();
	  memset(rx_Buff_V, 0, RX_BUFF_SIZE_VALVE);
	  HAL_UART_Receive_IT(&huart1, rx_Buff_V, 1);
	}
  // Main3
	if (huart->Instance == USART2)
	{
	  printf("Received from Main3 %d\r\n", rx_Buff_3[5]);
	  if(checkUART(rx_Buff_3, &ck_a_rx3, &ck_b_rx3))
	  {
	    if(rx_Buff_3[3] == MAIN1)
	    {
	      command = rx_Buff_3[5];
	    }else if(rx_Buff_3[3] == VALVE)
	    {
	      HAL_UART_Transmit(&huart1, &rx_Buff_3[5], 1, 10);
	      printf("SendToValve:%c\r\n", rx_Buff_3[5]);
	    }
	  }
	  flag = 0;
		MX_USART2_UART_Init();
		memset(rx_Buff_3, 0, RX_BUFF_SIZE_MAIN3);
		HAL_UART_Receive_IT(&huart2, rx_Buff_3, RX_BUFF_SIZE_MAIN3);
	}
  // PC
	if (huart->Instance == USART4)
	{
	  printf("Receive from PC:");

	  printf("%d", rx_Buff_PC[0]);
	  tx_Buff_V[0] = rx_Buff_PC[0];
	  printf("\r\n");
	    for(int i = 0; i < TX_BUFF_SIZE_VALVE; i++)
        {
          printf("%#x", tx_Buff_V[i]);
        }
	    printf("\r\n");
	  HAL_UART_Transmit(&huart1, tx_Buff_V, TX_BUFF_SIZE_VALVE, 10);
	  HAL_UART_Receive_IT(&huart4, rx_Buff_PC, 1);
	}
}

void send_VALVE(uint8_t *tx_Buff, uint8_t com){
    sendto = VALVE;
    sendfrom = MAIN1;

	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[3] = sendfrom;
	tx_Buff[4] = TX_BUFF_SIZE_VALVE;
    tx_Buff[5] = com;

	calculateChecksum(tx_Buff, TX_BUFF_SIZE_VALVE, &ck_a_tx3, &ck_b_tx3);
	tx_Buff[TX_BUFF_SIZE_VALVE - 2] = ck_a_tx3;
	tx_Buff[TX_BUFF_SIZE_VALVE - 1] = ck_b_tx3;
}

void send_MAIN3(uint8_t *tx_Buff){
	sendto = MAIN3;
	sendfrom = MAIN1;

    tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
    tx_Buff[1] = ADDRESS & 0xFF;
    tx_Buff[2] = sendto;
    tx_Buff[3] = sendfrom;
    tx_Buff[4] = TX_BUFF_SIZE_MAIN3;
    tx_Buff[5] = voltage_send;
    tx_Buff[6] = PHASE;
    tx_Buff[7] = TankPressure;

    calculateChecksum(tx_Buff, TX_BUFF_SIZE_MAIN3, &ck_a_tx3, &ck_b_tx3);
    tx_Buff[TX_BUFF_SIZE_MAIN3 - 2] = ck_a_tx3;
    tx_Buff[TX_BUFF_SIZE_MAIN3 - 1] = ck_b_tx3;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
