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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//void MX25L8006E_ReadData(uint32_t address, uint8_t *buffer, uint16_t length);
//void MX25L8006E_WriteEnable();
//void MX25L8006E_PageProgram(uint32_t address, uint8_t *data, uint16_t length);
//void MX25L8006E_WaitForWriteComplete();
void printHex(uint8_t *data, uint16_t length) ;
void MX25L8006E_SectorErase(uint32_t address);
void MX25L8006E_ReadID(uint8_t *id);
//uint8_t ReciveDataSpiControl(void);
uint8_t MX25L8006E_ReadStatus();
void MX25L8006E_WriteEnable(void);
void MX25L8006E_WaitForWriteComplete(void);
void MX25L8006E_PageProgram(uint32_t address, uint8_t *data, uint16_t length);
void MX25L8006E_WriteData(uint32_t start_address, uint8_t *data, uint32_t total_length);
void MX25L8006E_ReadData(uint32_t start_address, uint8_t *buffer, uint32_t length);
void MX25L8006E_EraseAll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MX25L8006E_Init(void) {
    HAL_SPI_MspInit(&hspi1);
    MX_SPI1_Init();
}

#define MX25L8006E_READ_DATA 0x03
#define MX25L8006E_WRITE_ENABLE 0x06
#define MX25L8006E_PAGE_PROGRAM 0x02
#define MX25L8006E_SECTOR_ERASE 0x20
#define MX25L8006E_READ_STATUS 0x05
#define MX25L8006E_READ_ID 0x9F

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
  /* USER CODE BEGIN 2 */
  HAL_SPI_MspInit(&hspi1);
//  uint8_t data[256];
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // CS=Hにしておく

    __HAL_SPI_ENABLE(&hspi1);
//  uint8_t id[3];
//  for(int i = 0; i < 3; i++) {
//	  id[i] = i;
//    }
//    MX25L8006E_ReadID(id);
//    printf("Device ID: 0x%02X 0x%02X 0x%02X\n", id[0], id[1], id[2]);
//    MX25L8006E_ReadID(id);
//    printf("Device ID: 0x%02X 0x%02X 0x%02X\n", id[0], id[1], id[2]);
  uint8_t writeData[256] = { 'H', 'e', 'l', 'l', 'o', 0x00 };  // 書き込み??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?ータ
  uint8_t readData[256] = {0};
  for(int i = 0; i < 256; i++) {
	  readData[i] = i;
  }

  uint8_t status = MX25L8006E_ReadStatus();
  printf("Status Register: 0x%02X\n", status);

  MX25L8006E_SectorErase(0x123456);
  HAL_Delay(5000);

//
//    uint8_t received_data = ReciveDataSpiControl();
//    HAL_UART_Transmit(&huart1, &received_data, 1, HAL_MAX_DELAY);
    MX25L8006E_PageProgram(0x123456, writeData, 256);
//  MX25L8006E_WaitForWriteComplete();

  HAL_Delay(5000);

  MX25L8006E_ReadData(0x123456, readData, 256);
  HAL_Delay(5000);

  HAL_UART_Transmit(&huart1, (uint8_t*)"Write Data: ", strlen("Write Data: "), HAL_MAX_DELAY);
//  printHex(writeData, 256);

  HAL_UART_Transmit(&huart1, (uint8_t*)"Read Data: ", strlen("Read Data: "), HAL_MAX_DELAY);
  printHex(readData, 256);

//  uint8_t data[31] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
//      uint8_t read_buffer[31];
//
//      // Erase all sectors
////      MX25L8006E_EraseAll();
//      MX25L8006E_SectorErase(0x000000);
//
//      // Write data to flash memory starting at address 0x000000
//      MX25L8006E_WriteData(0x000000, data, sizeof(data));
//
//      HAL_Delay(5000);
//
//      // Read data from flash memory starting at address 0x000000
//      MX25L8006E_ReadData(0x000000, read_buffer, sizeof(read_buffer));
//
//      HAL_Delay(5000);
//
//      printHex(read_buffer,sizeof(read_buffer));



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printf("looping\r\n");
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//
//void MX25L8006E_WriteEnable() {
//    uint8_t cmd = MX25L8006E_WRITE_ENABLE;
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
//    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High
//}
//void MX25L8006E_PageProgram(uint32_t address, uint8_t *data, uint16_t length) {
//    uint8_t cmd[4];
//    cmd[0] = MX25L8006E_PAGE_PROGRAM;
//    cmd[1] = (address >> 16) & 0xFF;
//    cmd[2] = (address >> 8) & 0xFF;
//    cmd[3] = address & 0xFF;
//
//    MX25L8006E_WriteEnable();
//
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
//    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
//    HAL_SPI_Transmit(&hspi1, data, length, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High
//    MX25L8006E_WaitForWriteComplete();
//}
uint8_t MX25L8006E_ReadStatus() {
    uint8_t cmd = MX25L8006E_READ_STATUS;
    uint8_t status;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High
    return status;
}

//void MX25L8006E_WaitForWriteComplete() {
//    while (MX25L8006E_ReadStatus() & 0x01) {
//    	HAL_Delay(1);
//    }
//}
//void MX25L8006E_ReadData(uint32_t address, uint8_t *buffer, uint16_t length) {
//    uint8_t cmd[4];
//    cmd[0] = MX25L8006E_READ_DATA;
//    cmd[1] = (address >> 16) & 0xFF;
//    cmd[2] = (address >> 8) & 0xFF;
//    cmd[3] = address & 0xFF;
//
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
//    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
//    HAL_SPI_Receive(&hspi1, buffer, length, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High
//}
void printHex(uint8_t *data, uint16_t length) {
    for(int i = 0; i < length; i++) {
        char hex[3];
        sprintf(hex, "%02X", data[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)hex, 2, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t*)" ", 1, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}
void MX25L8006E_SectorErase(uint32_t address) {
    uint8_t cmd[4];
    cmd[0] = MX25L8006E_SECTOR_ERASE;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    MX25L8006E_WriteEnable();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High

    //MX25L8006E_WaitForWriteComplete(); // 消去完�?を�?つ
}
//uint8_t ReciveDataSpiControl(void) {
//    uint8_t Data = 0;
//    // 送信バッファが空になるまで??��?��???��?��?
//    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
//
//    uint8_t dummy_data = 0xFF;
//    HAL_SPI_TransmitReceive(&hspi1, &dummy_data, &Data, 1, HAL_MAX_DELAY);
//
//    // シフトされたデータを返す
//    return Data;
//}
void MX25L8006E_ReadID(uint8_t *id) {
    uint8_t cmd = MX25L8006E_READ_ID;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
    HAL_SPI_TransmitReceive(&hspi1, &cmd, id, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High
}
void MX25L8006E_WriteEnable(void) {
      uint8_t cmd = MX25L8006E_WRITE_ENABLE;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
      HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High
  }
  void MX25L8006E_WaitForWriteComplete(void) {
      uint8_t cmd = MX25L8006E_READ_STATUS;
      uint8_t status = 0;

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
      do {
          HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
          HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
      } while (status & 0x01); // Wait until the write in progress bit is cleared
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High
  }
  void MX25L8006E_PageProgram(uint32_t address, uint8_t *data, uint16_t length) {
      uint8_t cmd[4];

      MX25L8006E_WriteEnable();

      cmd[0] = MX25L8006E_PAGE_PROGRAM;
      cmd[1] = (address >> 16) & 0xFF;
      cmd[2] = (address >> 8) & 0xFF;
      cmd[3] = address & 0xFF;

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
      HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
      HAL_SPI_Transmit(&hspi1, data, length, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High

      MX25L8006E_WaitForWriteComplete();
  }

  void MX25L8006E_WriteData(uint32_t start_address, uint8_t *data, uint32_t total_length) {
      uint32_t address = start_address;
      uint32_t remaining = total_length;
      uint32_t write_length;

      while (remaining > 0) {
          // Calculate the length to write in this iteration (max 31 bytes)
          write_length = (remaining < 31) ? remaining : 31;

          // Ensure the data doesn't cross a page boundary (256 bytes per page)
          if ((address % 256) + write_length > 256) {
              write_length = 256 - (address % 256);
          }

          // Write the data
          MX25L8006E_PageProgram(address, data, write_length);

          // Update the address and remaining length
          address += write_length;
          data += write_length;
          remaining -= write_length;
      }
  }

  void MX25L8006E_ReadData(uint32_t start_address, uint8_t *buffer, uint32_t length) {
      uint8_t cmd[4];
      uint32_t address = start_address;
      uint32_t remaining = length;
      uint32_t read_length;

      while (remaining > 0) {
          // Calculate the length to read in this iteration (max 31 bytes)
          read_length = (remaining < 31) ? remaining : 31;

          // Send read command and address
          cmd[0] = MX25L8006E_READ_DATA;
          cmd[1] = (address >> 16) & 0xFF;
          cmd[2] = (address >> 8) & 0xFF;
          cmd[3] = address & 0xFF;

          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
          HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
          HAL_SPI_Receive(&hspi1, buffer, read_length, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High

          // Update the address and remaining length
          address += read_length;
          buffer += read_length;
          remaining -= read_length;
      }
  }

  void MX25L8006E_EraseAll(void) {
      uint8_t cmd = 0xC7; // Chip Erase command

      MX25L8006E_WriteEnable();

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
      HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High

      MX25L8006E_WaitForWriteComplete();
  }
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,HAL_MAX_DELAY);
  return len;
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
