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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_Buff_1[RX_BUFF_SIZE_RM92A];
uint8_t rx_Buff_2[RX_BUFF_SIZE_PC];

uint8_t tx_Buff_1[TX_BUFF_SIZE_RM92A];


float ftime_rx;
float ftime;
float pre_ftime = 0.000;
float lat;
float lat_min;
float lat_sec;
float lon;
float lon_min;
float lon_sec;
float alt;
float pres;
float temp;
float volt;
float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float tpres;

int PHASE;
int mtime;
int mtime_h;
int mtime_m;
int mtime_s;
int judg;
int fpin;
int VPHASE;
int fcount = 0;

uint8_t Buff_size;
uint8_t ck_a_rx_1, ck_b_rx_1;   // for checksum
uint8_t ck_a_rx_2, ck_b_rx_2;   // for checksum
uint8_t ck_a_tx_1, ck_b_tx_1;   // for checksum
int flag;   // use as bool function. True:1, False:0

SEND_TO sendto, sendfrom, wantto;

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, rx_Buff_1,  RX_BUFF_SIZE_RM92A);
  HAL_UART_Receive_IT(&huart2, rx_Buff_2, RX_BUFF_SIZE_PC);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,HAL_MAX_DELAY);
  return len;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1) {
    	printf("Receive from RM-92A\r\n");
//		for (int i = 0; i < 32; i++) {
//			printf("array[%d] = %d\r\n", i, rx_Buff_1[i]);
//		}

    	checkUART(rx_Buff_1, &ck_a_rx_1, &ck_b_rx_1);
    	printf("%d\r\n",flag);
    	if (flag == 1){
    		print_PC(rx_Buff_1);
    	}
//
//    	print_PC(rx_Buff_1);

    	MX_USART1_UART_Init();
		memset(rx_Buff_1, 0, RX_BUFF_SIZE_RM92A);
		HAL_UART_Receive_IT(&huart1, rx_Buff_1, RX_BUFF_SIZE_RM92A);
	}

	if (huart->Instance == USART2) {
			printf("Receive from PC\r\n");

			send_rm(tx_Buff_1, rx_Buff_2);

			MX_USART2_UART_Init();
			memset(rx_Buff_2, 0, RX_BUFF_SIZE_PC);
			memset(tx_Buff_1, 0, TX_BUFF_SIZE_RM92A);
			HAL_UART_Receive_IT(&huart2, rx_Buff_2, RX_BUFF_SIZE_PC);
		}
}


void print_PC(uint8_t *rx_Buff){

	PHASE = (int)rx_Buff[5];

	mtime = ((uint16_t)rx_Buff[6] << 8) | (int)rx_Buff[7];

	mtime_h = mtime / 3600;
	mtime_m = (mtime - mtime_h * 3600) / 60;
	mtime_s = mtime - mtime_h * 3600 - mtime_m * 60;

	ftime_rx = (float)((uint16_t)rx_Buff[8] << 8 | rx_Buff[9]) / 100;
	if(ftime_rx < pre_ftime){
		fcount ++;
	}
	//ftime = ftime_rx + fcount * 655.35;
	ftime = ftime_rx;

	lat_min = rx_Buff[10] + (float)(rx_Buff[11])/100;
	lat = LAT_OFFSET + lat_min/60;

	lon_min = rx_Buff[12] + (float)(rx_Buff[13])/100;
	lon = LON_OFFSET + lon_min/60;


	alt = ((uint16_t)rx_Buff[14] << 8 | rx_Buff[15]) + (float)rx_Buff[16] / 100;
	pres = (float)((uint32_t)rx_Buff[17] << 16 | (uint16_t)rx_Buff[18] << 8 | rx_Buff[19]) / 100;
	temp = rx_Buff[20] + (float)((rx_Buff[21] & 0b11110000) >> 4) / 10;
	judg = (rx_Buff[21] & 0b00001100) >> 2;
	fpin = (rx_Buff[21] & 0b00000010) >> 1;
	volt = (float)rx_Buff[22] / 10;

	if (rx_Buff[23] > 128){
		acc_x = rx_Buff[23] - 256 - (float)rx_Buff[24] / 100;
	}else{
		acc_x = rx_Buff[23] + (float)rx_Buff[24] / 100;
	}

	if (rx_Buff[25] > 128){
		acc_y = rx_Buff[25] -256 - (float)rx_Buff[26] / 100;
	}else{
		acc_y = rx_Buff[25] + (float)rx_Buff[26] / 100;
	}

	if (rx_Buff[27] > 128){
		acc_z = rx_Buff[27] -256 - (float)rx_Buff[28] / 100;
	}else{
		acc_z = rx_Buff[27] + (float)rx_Buff[28] / 100;
	}

	if (rx_Buff[29] > 128){
		gyro_x = rx_Buff[29] -256 - (float)rx_Buff[30] / 100;
	}else{
		gyro_x = rx_Buff[29] + (float)rx_Buff[30] / 100;
	}

	if (rx_Buff[31] > 128){
		gyro_y = rx_Buff[31] -256 - (float)rx_Buff[32] / 100;
	}else{
		gyro_y = rx_Buff[31] + (float)rx_Buff[32] / 100;
	}

	if (rx_Buff[33] > 128){
		gyro_z = rx_Buff[33] -256 - (float)rx_Buff[34] / 100;
	}else{
		gyro_z = rx_Buff[33] + (float)rx_Buff[34] / 100;
	}

	VPHASE = rx_Buff[35];
	tpres = (float)rx_Buff[36] / 10;

	pre_ftime = ftime_rx;


	printf(" ======== PLANET-Q NSE2024 ========\r\n");
	printf(" PHASE \r\n");
	printf(" 0 : SAFETY\r\n");
	printf(" 1 : READY\r\n");
	printf(" 2 : BURNING\r\n");
	printf(" 3 : FLIGHT\r\n");
	printf(" 4 : SEP\r\n");
	printf(" 5 : LANDED\r\n");
	printf(" 6 : EMERGENCY\r\n");
	printf("\r\n");

	printf(" PHASE : %d\r\n", PHASE);
	printf("\r\n");

	printf(" MISSION TIME : %d h %d m %d s\r\n", mtime_h, mtime_m, mtime_s);
	printf(" FLIGHT TIME : \t%.2f [s]\r\n", ftime);
	printf("\r\n");

	if(fpin == 0){
		printf(" Flight Pin is connected... \r\n");
	}else if(fpin == 1){
		printf(" Flight Pin is out! \r\n");
	}
	printf("\r\n");

	printf(" JUDGMENT :");
	if(judg == 0){
		printf(" UNACTIVATED \r\n");
	}else if(judg == 1){
		printf(" TIMER \r\n");
	}else if(judg == 2){
		printf(" PRESSURE \r\n");
	}else if(judg == 3){
		printf(" UPLINK COMMAND \r\n");
	}
	printf("\r\n");

	printf(" LATITUDE : \t%.4f [°] \r\n", lat);
	printf(" LONGITUDE : \t%.4f [°] \r\n", lon);
	printf(" ALTITUDE : \t%.2f [m] \r\n", alt);
	printf(" PRESSURE : \t%.2f [hPa] \r\n", pres);
	printf(" TEMPERATURE : \t%.1f [℃] \r\n", temp);
	printf(" VOLTAGE : \t%.1f [V] \r\n", volt);
	printf("\r\n");

	printf("ACCERALATION [m/s^2]\r\n");
	printf(" x: %.2f, \t y: %.2f, \t z: %.2f\r\n", acc_x, acc_y, acc_z);
	printf("GYRO [deg/s]\r\n");
	printf(" x: %.2f, \t y: %.2f, \t z: %.2f\r\n", gyro_x, gyro_y, gyro_z);
	printf("\r\n");
	printf("\r\n");

	printf(" ------ NSE2024 VALVE ------ \r\n");
	printf(" PHASE \r\n");
	printf(" 0 : SAFETY\r\n");
	printf(" 1 : READY\r\n");
	printf(" 2 : OPEN\r\n");
	printf(" 3 : CLOSE\r\n");
	printf(" 4 : EMERGENCY\r\n");
	printf("\r\n");

	printf(" PHASE : %d\r\n", VPHASE);
	printf("\r\n");

	printf(" TANK PRESSURE : %.1f [MPa]\r\n", tpres);
	printf("\r\n");
	printf("\r\n");
	printf("\r\n");

}

void send_rm(uint8_t *tx_Buff,uint8_t *rx_Buff){
	char input[2];
	sendto = MAIN2;

	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[4] = TX_BUFF_SIZE_RM92A;
	tx_Buff[5] = rx_Buff[1] - 48;  //ASCII コードでは'0'?��?48,'1'?��?49で表される�?

	input[0] = rx_Buff[0];
	input[1] = rx_Buff[1];

	if (input[0] == 'V'){
		wantto = VALVE;
		printf("Send to VALVE !\r\n");
	}else{
		wantto = MAIN3;
		printf("Send to MAIN3 !\r\n");
	}
	tx_Buff[3] = wantto;

	calculateChecksum(tx_Buff, TX_BUFF_SIZE_RM92A, &ck_a_tx_1, &ck_b_tx_1);
	tx_Buff[TX_BUFF_SIZE_RM92A - 2] = ck_a_tx_1;
	tx_Buff[TX_BUFF_SIZE_RM92A - 1] = ck_b_tx_1;

	HAL_UART_Transmit(&huart1, tx_Buff, TX_BUFF_SIZE_RM92A, 0xFFFF);

	printf("send : %d\r\n", tx_Buff[5]);
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
	if (((rx_Buff[0] << 8) | (rx_Buff[1])) == ADDRESS || rx_Buff[2] == 4){
		//printf("address is ok!");
        Buff_size = rx_Buff[4];
		calculateChecksum(rx_Buff, Buff_size, ck_a, ck_b);
		flag = 1;


//		if (*ck_a == rx_Buff[Buff_size - 2] || *ck_b == rx_Buff[Buff_size - 1]){
//			flag = 1;
//		}else{
//			printf("no...");
//			flag = 0;
//		}

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
