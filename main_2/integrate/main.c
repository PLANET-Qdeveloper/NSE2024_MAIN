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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	VALVE,
	MAIN1,
	MAIN2,
	MAIN3,
	GROUND
} SEND_TO;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDRESS 2024   // inspire from this year

#define TX_BUFF_SIZE_RM92A 39   // buffer size send to RM92A(downlink)
#define TX_BUFF_SIZE_MAIN3 8  // buffer size send to main3
#define RX_BUFF_SIZE_RM92A 8  // buffer size receive from RM92A(uplink)
#define RX_BUFF_SIZE_MAIN3 39  // buffer size receive from main3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void send_rm(uint8_t *tx_Buff, uint8_t *rx_Buff);  // send to RM-92A
void send_main3(uint8_t *tx_Buff, uint8_t *rx_Buff);  // send to main3
void calculateChecksum(uint8_t *Buff, uint8_t Buff_size, uint8_t *ck_a, uint8_t *ck_b);   // for checksum
void checkUART(uint8_t *rx_Buff, uint8_t *ck_a, uint8_t *ck_b);   // if True, it adjusts Buff_size, PHASE

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_Buff_2[RX_BUFF_SIZE_MAIN3];   // receive from main3
uint8_t rx_Buff_3[RX_BUFF_SIZE_RM92A];   // receive from RM-92A(uplink)

uint8_t tx_Buff_1[TX_BUFF_SIZE_RM92A];   // send to RM-92A(downlink)
uint8_t tx_Buff_2[TX_BUFF_SIZE_MAIN3];   // send to main3

uint8_t Buff_ex[39] = {7,232,4,4,39,
		3,255,255,3,17,
		0,123,38,148,4,
		224,32,1,86,102,
		19,98,63,9,80,
		9,80,9,80,100,
		20,100,21,100,22,
		2,13,98,254};

uint8_t PHASE = 0;   // PHASE
uint8_t Buff_size;
uint8_t count;   // adjustment of downlink rate
uint8_t ck_a_rx_2, ck_b_rx_2;   // for checksum
uint8_t ck_a_rx_3, ck_b_rx_3;   // for checksum
uint8_t ck_a_tx_2, ck_b_tx_2;   // for checksum
uint8_t ck_a_tx_3, ck_b_tx_3;   // for checksum
int flag = 1;   // use as bool function. True:1, False:0

SEND_TO sendto, wantto;


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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, rx_Buff_2, RX_BUFF_SIZE_MAIN3);
  HAL_UART_Receive_IT(&huart3, rx_Buff_3, RX_BUFF_SIZE_RM92A);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printf("PAHSE : %d\r\n", PHASE);
	  printf("loop\r\n");



/*	  checkUART(Buff_ex, &ck_a_rx_2, &ck_b_rx_2);
	  printf("flag : %d\r\n", flag);

	  if (flag == 1){
		PHASE = Buff_ex[5];

		if (PHASE == 0){
			count += 1;
		}else{
			count += 5;
		}

		if (count >= 5){
	  		send_rm(tx_Buff_1, Buff_ex);
	  		count = 0;
	  	}
	  }*/

	  //flag = 0;


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

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart4,(uint8_t *)ptr,len,HAL_MAX_DELAY);
  return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		printf("Receive from MAIN3\r\n");
		checkUART(rx_Buff_2, &ck_a_rx_2, &ck_b_rx_2);
		printf("flag : %d\r\n", flag);
		flag = 1;

		if (flag == 1){
			PHASE = rx_Buff_2[5];
			if (PHASE == 0){
				count += 1;
			}else{
				count += 5;
			}

			if (count >= 5){
				send_rm(tx_Buff_1, rx_Buff_2);
				count = 0;
			}
		}

		flag = 0;
		MX_USART2_UART_Init();
		memset(rx_Buff_2, 0, RX_BUFF_SIZE_MAIN3);
		memset(tx_Buff_1, 0, TX_BUFF_SIZE_RM92A);
		HAL_UART_Receive_IT(&huart2, rx_Buff_2, RX_BUFF_SIZE_MAIN3);
	}

 	if (huart->Instance == USART3) {
		printf("Receive from GROUND\r\n");
		for (int i = 0; i < RX_BUFF_SIZE_RM92A; i++){
			printf("data[%d] : %d\r\n", i,rx_Buff_3[i]);
		}

		checkUART(rx_Buff_3, &ck_a_rx_3, &ck_b_rx_3);
		printf("flag:%d\r\n", flag);

		if (flag == 1){
			send_main3(tx_Buff_2, rx_Buff_3);
		}


		flag = 0;
		MX_USART3_UART_Init();
		memset(rx_Buff_3, 0, RX_BUFF_SIZE_RM92A);
		memset(tx_Buff_2, 0, TX_BUFF_SIZE_MAIN3);
		HAL_UART_Receive_IT(&huart3, rx_Buff_3, RX_BUFF_SIZE_RM92A);
	}
}

void send_rm(uint8_t *tx_Buff,uint8_t *rx_Buff){
	sendto = GROUND;
	wantto = GROUND;
	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[3] = wantto;
	tx_Buff[4] = TX_BUFF_SIZE_RM92A;

	for (int i = 0; i < (TX_BUFF_SIZE_RM92A - 2); i++){
		tx_Buff[i+5] = rx_Buff[i+5];
	}

	calculateChecksum(tx_Buff, TX_BUFF_SIZE_RM92A, &ck_a_tx_3, &ck_b_tx_3);
	tx_Buff[-2] = ck_a_tx_3;
	tx_Buff[-1] = ck_b_tx_3;

	HAL_UART_Transmit(&huart1, tx_Buff, TX_BUFF_SIZE_RM92A, 0xFFFF);

}

void send_main3(uint8_t *tx_Buff, uint8_t *rx_Buff){
	sendto = MAIN3;
	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[3] = rx_Buff[3];
	tx_Buff[4] = TX_BUFF_SIZE_MAIN3;

	for (int i = 0; i < (TX_BUFF_SIZE_RM92A - 2); i++){
			tx_Buff[i+4] = rx_Buff[i+5];
	}

	calculateChecksum(tx_Buff, TX_BUFF_SIZE_MAIN3, &ck_a_tx_2, &ck_b_tx_2);
	tx_Buff[-2] = ck_a_tx_2;
	tx_Buff[-1] = ck_b_tx_2;

	HAL_UART_Transmit(&huart2, tx_Buff, TX_BUFF_SIZE_MAIN3, 0xFFFF);
	printf("send to main3!\r\n");

}


void calculateChecksum(uint8_t *Buff, uint8_t Buff_size, uint8_t *ck_a, uint8_t *ck_b){
	*ck_a = 0;
    *ck_b = 0;

    for (int i = 0; i < (Buff_size - 2); i++) {
        *ck_a = (*ck_a + Buff[i]) & 0xFF;
        *ck_b = (*ck_b + *ck_a) & 0xFF;
    }
}


void checkUART(uint8_t *rx_Buff, uint8_t *ck_a, uint8_t *ck_b){
	if (((rx_Buff[0] << 8) | (rx_Buff[1])) == ADDRESS || rx_Buff[2] == 2){
		printf("ADDRESS is OK\r\n");

        Buff_size = rx_Buff[4];
		calculateChecksum(rx_Buff, Buff_size, ck_a, ck_b);


		if (*ck_a == rx_Buff[Buff_size - 2] || *ck_b == rx_Buff[Buff_size - 1]){
			flag = 1;
			printf("GOOD!\r\n");
		}else{
			flag = 0;
			printf("CK...\r\n");
		}

	}else{
		flag = 0;
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
