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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bme280/bmp280.h"
#include "bno055/bno055_stm32.h"
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
PHASE currentPhase = SAFETY;
PHASE previousPhase = SAFETY;
SEND_TO sendto, sendfrom, wantto;

bno055_vector_t accelerometer;
bno055_vector_t gyroscope;
BMP280_HandleTypedef bmp280;
float pressure, temperature, humidity;
float previous_pressure;

uint8_t rx_Buff_2[RX_BUFF_SIZE_MAIN2];   // receive from main2
uint8_t rx_Buff_3[RX_BUFF_SIZE_MAIN1];   // receive from main2

uint8_t tx_Buff_2[TX_BUFF_SIZE_MAIN2];   // send to main2
uint8_t tx_Buff_3[TX_BUFF_SIZE_MAIN1];   // send to main1

char rx_Buff_GPS[GPS_BUFFER];

uint8_t command;
uint8_t Buff_size;
uint8_t count;   // adjustment of downlink rate
uint8_t ck_a, ck_b;   // for checksum
int flag;   // use as bool function. True:1, False:0

uint16_t mission_time = 0;
uint32_t flight_time = 0;
uint8_t latitude[2];
uint8_t longitude[2];
uint8_t altitude[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  printf("bmp280 setup");
  bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;

	while (!bmp280_init(&bmp280, &bmp280.params))
		{
			printf("BME280 initialization failed\r\n");
			HAL_Delay(2000);
		}

	  printf("main3 setup");
  MAIN3_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (currentPhase != previousPhase) {


		  switch (currentPhase) {
		  	  case SAFETY:
				  __HAL_TIM_SET_AUTORELOAD(&htim6, SAFETY_PERIOD - 1);
				  break;
		  	  case READY:
				  __HAL_TIM_SET_AUTORELOAD(&htim6, READY_PERIOD - 1);
				  break;
			  case BURNING:
				  __HAL_TIM_SET_AUTORELOAD(&htim6, BURNING_PERIOD - 1);
				  HAL_TIM_Base_Start(&htim14);//Timer1
				  break;
			  case FLIGHT:
				  __HAL_TIM_SET_AUTORELOAD(&htim6, FLIGHT_PERIOD - 1);
				  __HAL_TIM_SET_AUTORELOAD(&htim14, TOP_DETECT_PERIOD - 1);//Timer1 top detect
				  HAL_TIM_Base_Start(&htim17);//bmp280 top detect
				  break;
			  case SEP:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				  __HAL_TIM_SET_AUTORELOAD(&htim6, SEP_PERIOD - 1);
				  __HAL_TIM_SET_AUTORELOAD(&htim17, MOSFET_PERIOD - 1);//Timer2
				  break;
			  case LANDED:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
				  __HAL_TIM_SET_AUTORELOAD(&htim6, LANDED_PERIOD - 1);
				  break;
			  case EMERGENCY:
				  __HAL_TIM_SET_AUTORELOAD(&htim6, EMERGENCY_PERIOD - 1);
				  break;
		  }
	}else{
		switch (currentPhase) {
			  case READY:
				  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET){
					  currentPhase = BURNING;
				  }

				  break;
			  case BURNING:
				  break;
			  case FLIGHT:
				  break;
			  case SEP:
				  break;
			  case LANDED:
				  break;
			  case EMERGENCY:
				  break;
		}
		HAL_Delay(10);
	}

	  previousPhase = currentPhase;

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
void MAIN3_Init() {
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

//	bmp280_init_default_params(&bmp280.params);
//	bmp280.addr = BMP280_I2C_ADDRESS_0;
//	bmp280.i2c = &hi2c2;

//	const char setRate10Hz[] = "$PMTK220,100*2F\r\n";
//	HAL_UART_Transmit(&huart4, (uint8_t*)setRate10Hz, strlen(setRate10Hz), HAL_MAX_DELAY);
//	const char setGGAOnly[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
//	HAL_UART_Transmit(&huart4, (uint8_t*)setGGAOnly, strlen(setGGAOnly), HAL_MAX_DELAY);



    HAL_UART_Receive_IT(&huart2, rx_Buff_2, RX_BUFF_SIZE_MAIN2);
    HAL_UART_Receive_IT(&huart3, rx_Buff_3, RX_BUFF_SIZE_MAIN1);
	HAL_UART_Receive_IT(&huart4, rx_Buff_GPS, GPS_BUFFER - 1);

//	while (!bmp280_init(&bmp280, &bmp280.params))
//	{
//		printf("BME280 initialization failed\r\n");
//		HAL_Delay(2000);
//	}
}

void measure(){
	accelerometer = bno055_getVectorAccelerometer();
	gyroscope = bno055_getVectorGyroscope();
	if(!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)){
		temperature = 0.0;
		pressure = 0.0;
		humidity = 0.0;
	}
}

void record(){
	for (size_t i = 0; i < 39; i++) {
	        printf("%u ", tx_Buff_2[i]);
	    }
	    printf("\r\n");
}

void top_detect(){
	int count = 0;
	if(previous_pressure < pressure){
		count ++;
		if(count >= 5){
			currentPhase = SEP;
		}
	}else{
		count = 0;
	}
	previous_pressure = pressure;
}

void processGPSData(char *buffer){
	char *ggaMessage = strstr(buffer, "$GPGGA");
	char* rest = ggaMessage;
	char token[20];

	strtok_r(rest, ",", &rest);  // $GPGGA
	strtok_r(rest, ",", &rest);  // time

	char* latToken = strtok_r(rest, ",", &rest);  // lat
	if (latToken != NULL) {
		strncpy(token, latToken, sizeof(token) - 1);
		token[-1] = '\0';
		uint8_t latitude_int;
		sscanf(token, "%2d", &latitude_int);
		if(latitude_int == 0){
			latitude[0] = 0x55;
			latitude[1] = 0x55;
		}else{
			sscanf(token + 2, "%2d", &latitude[0]);
			sscanf(token + 5, "%2d", &latitude[1]);
		}
	}else{
		latitude[0] = 0x55;
		latitude[1] = 0x55;
	}

	strtok_r(rest, ",", &rest);  // N/S indicator

	char* lonToken = strtok_r(rest, ",", &rest);  // lon
	if (lonToken != NULL) {
		strncpy(token, lonToken, sizeof(token) - 1);
		token[-1] = '\0';
		uint8_t longitude_int;
		sscanf(token, "%3d", &longitude_int);
		if(longitude_int == 0){
			longitude[0] = 0x55;
			longitude[1] = 0x55;
		}else{
		sscanf(token + 3, "%2d", &longitude[0]);
		sscanf(token + 6, "%2d", &longitude[1]);
		}
	}else{
		longitude[0] = 0x55;
		longitude[1] = 0x55;
	}

	strtok_r(rest, ",", &rest);  // E/W indicator
	strtok_r(rest, ",", &rest);  // fix quality
	strtok_r(rest, ",", &rest);  // number of satellites
	char* altToken = strtok_r(rest, ",", &rest);  // horizontal dilution of precision
	if (altToken != NULL) {
		double altitude_double = atof(altToken);
		uint16_t altitude_int = (int)altitude_double;
		uint8_t altitude_decimal = (int)((altitude_double - altitude_int) * 100);
		altitude[0] = (altitude_int >> 8) & 0xFF;
		altitude[1] = altitude_int & 0xFF;
		altitude[2] = altitude_decimal & 0xFF;
	}else{
		memset(altitude, 0, 3);
	}
}

//Timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        if(currentPhase==SAFETY || currentPhase==READY){
        	measure();
        }
        else{
        	measure();
			record();
        }
    }else if (htim->Instance == TIM7) {
    	send_GROUND();
	}else if (htim->Instance == TIM14) {
    	if(currentPhase == BURNING){
    		currentPhase = FLIGHT;
    	}else if(currentPhase == FLIGHT){
    		currentPhase = SEP;
    	}
	}else if (htim->Instance == TIM15) {
		flight_time ++;
    }else if (htim->Instance == TIM16) {
		mission_time ++;
	}else if (htim->Instance == TIM17) {
    	if(currentPhase == FLIGHT){
    		top_detect();
    	}else if(currentPhase == SEP){
    		currentPhase = LANDED;
    	}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
 	if (huart->Instance == USART2) {
		printf("Receive from MAIN2\r\n");
		checkUART(rx_Buff_2);

		if (flag == 1){
            command = rx_Buff_2[5];

            if (rx_Buff_2[3] == MAIN3){

                switch(command){
                    case 0:
                        switch(currentPhase){
                            case READY:
                                currentPhase = SAFETY;
                                break;

                            case BURNING:
                                currentPhase = READY;
                                break;

                            case LANDED:
                                currentPhase = SEP;
                                break;
                        }
                        break;

                    case 1:
                        switch(currentPhase){
                            case SAFETY:
                                currentPhase = READY;
                                break;

                            case FLIGHT:
                                currentPhase = SEP;
                                break;

                            case LANDED:
                                currentPhase = EMERGENCY;
                                break;
                        }

                        break;

                    case 6:
                        if (currentPhase != SAFETY){
                            currentPhase = EMERGENCY;
                        }
                        break;
                }

            }else if (rx_Buff_2[3] == VALVE){
                send_VALVE(tx_Buff_3, command);

            }else if (rx_Buff_2[3] == MAIN1){
                sendfrom = MAIN2;

                send_MAIN1(tx_Buff_3, sendfrom, command);

            }


		}

		flag = 0;
		MX_USART2_UART_Init();
		memset(rx_Buff_2, 0, RX_BUFF_SIZE_MAIN2);
		HAL_UART_Receive_IT(&huart2, rx_Buff_2, RX_BUFF_SIZE_MAIN2);
	}

    if (huart->Instance == USART3) {
        printf("Receive from MAIN1\r\n");
		checkUART(rx_Buff_3);

        if (flag == 1){
            if (rx_Buff_3[3] == MAIN1){
                sendfrom = MAIN3;
                command = rx_Buff_3[5];

                send_MAIN1(tx_Buff_3, sendfrom, command);

            }else if (rx_Buff_2[3] == MAIN2){
                sendfrom = MAIN1;
                command = rx_Buff_3[5];

                send_MAIN2(tx_Buff_2, sendfrom, command);

            }else if (rx_Buff_2[3] == VALVE){
                printf("not yet");
            }
        }

        flag = 0;
		MX_USART3_UART_Init();
		memset(rx_Buff_3, 0, RX_BUFF_SIZE_MAIN1);
        HAL_UART_Receive_IT(&huart3, rx_Buff_3, RX_BUFF_SIZE_MAIN1);

    }

    if (huart->Instance == USART4) {
	    rx_Buff_GPS[GPS_BUFFER - 1] = '\0';
	    processGPSData(rx_Buff_GPS);
	    MX_USART4_UART_Init();
	    memset(rx_Buff_GPS, 0, GPS_BUFFER);
       	HAL_UART_Receive_IT(&huart4, rx_Buff_GPS, GPS_BUFFER - 1);

       }
}


void send_MAIN2(uint8_t *tx_Buff, SEND_TO want_from, uint8_t com){
	sendto = MAIN2;

	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[3] = want_from;
	tx_Buff[4] = TX_BUFF_SIZE_MAIN2;

	if (want_from == GROUND){

    }else if (want_from == MAIN1){
        tx_Buff[5] = com;

    }


	calculateChecksum(tx_Buff, TX_BUFF_SIZE_MAIN2, &ck_a, &ck_b);
	tx_Buff[-2] = ck_a;
	tx_Buff[-1] = ck_b;

}

void send_GROUND(){
	int temp_decimal_first_digit = (int)(temperature * 10) % 10;
	uint8_t flight_pin_status = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) ? 1 : 0;


	tx_Buff_2[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff_2[1] = ADDRESS & 0xFF;
	tx_Buff_2[2] = MAIN2;
	tx_Buff_2[3] = GROUND;
	tx_Buff_2[4] = TX_BUFF_SIZE_MAIN2;
	tx_Buff_2[5] = currentPhase;
	tx_Buff_2[6] = (mission_time >> 8) & 0xFF;
	tx_Buff_2[7] = mission_time & 0xFF;
	tx_Buff_2[8] = (flight_time >> 8) & 0xFF;
	tx_Buff_2[9] = flight_time & 0xFF;
	tx_Buff_2[10] = latitude[0];
	tx_Buff_2[11] = latitude[1];
	tx_Buff_2[12] = longitude[0];
	tx_Buff_2[13] = longitude[1];
	tx_Buff_2[14] = altitude[0];
	tx_Buff_2[15] = altitude[1];
	tx_Buff_2[16] = altitude[2];
	tx_Buff_2[17] = ((int)pressure >> 16) & 0xFF;
	tx_Buff_2[18] = ((int)pressure >> 8) & 0xFF;
	tx_Buff_2[19] = (int)pressure & 0xFF;
	tx_Buff_2[20] =(int)temperature;
	tx_Buff_2[21] |= (temp_decimal_first_digit & 0x0F) << 4;
	tx_Buff_2[21] |= (rx_Buff_3[5] & 0x03) << 2;
	tx_Buff_2[21] |= (flight_pin_status & 0x01) << 1;
	tx_Buff_2[22] =0;
	tx_Buff_2[23] =(int)accelerometer.x;
	tx_Buff_2[24] =accelerometer.x - (int)accelerometer.x;
	tx_Buff_2[25] =(int)accelerometer.y;
	tx_Buff_2[26] =accelerometer.y - (int)accelerometer.y;
	tx_Buff_2[27] =(int)accelerometer.z;
	tx_Buff_2[28] =accelerometer.z - (int)accelerometer.z;
	tx_Buff_2[29] =(int)gyroscope.x;
	tx_Buff_2[30] =gyroscope.x - (int)gyroscope.x;
	tx_Buff_2[31] =(int)gyroscope.y;
	tx_Buff_2[32] =gyroscope.y - (int)gyroscope.y;
	tx_Buff_2[33] =(int)gyroscope.z;
	tx_Buff_2[34] =gyroscope.z - (int)gyroscope.z;
	tx_Buff_2[35] =0;
	tx_Buff_2[36] =0;

	calculateChecksum(tx_Buff_2, TX_BUFF_SIZE_MAIN2, &ck_a, &ck_b);
	tx_Buff_2[-2] = ck_a;
	tx_Buff_2[-1] = ck_b;

	HAL_UART_Transmit(&huart2, tx_Buff_2, TX_BUFF_SIZE_MAIN2, HAL_MAX_DELAY);

}


void send_VALVE(uint8_t *tx_Buff, uint8_t com){
	sendto = MAIN1;
	wantto = VALVE;

	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[3] = wantto;
	tx_Buff[4] = TX_BUFF_SIZE_MAIN1;
    tx_Buff[5] = com;

	calculateChecksum(tx_Buff, TX_BUFF_SIZE_MAIN1, &ck_a, &ck_b);
	tx_Buff[-2] = ck_a;
	tx_Buff[-1] = ck_b;

}


void send_MAIN1(uint8_t *tx_Buff, SEND_TO from, uint8_t com){
	sendto = MAIN1;

	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[3] = from;
	tx_Buff[4] = TX_BUFF_SIZE_MAIN1;
    tx_Buff[5] = com;

	calculateChecksum(tx_Buff, TX_BUFF_SIZE_MAIN1, &ck_a, &ck_b);
	tx_Buff[-2] = ck_a;
	tx_Buff[-1] = ck_b;

}


void calculateChecksum(uint8_t *Buff, uint8_t Buff_size, uint8_t *ck_a, uint8_t *ck_b) {
	*ck_a = 0;
    *ck_b = 0;

    for (int i = 0; i < (Buff_size - 2); i++) {
        *ck_a = (*ck_a + Buff[i]) & 0xFF;
        *ck_b = (*ck_b + *ck_a) & 0xFF;
    }
}


void checkUART(uint8_t *rx_Buff){
	if (((rx_Buff[0] << 8) | (rx_Buff[1])) == ADDRESS || rx_Buff[2] == 2){
		Buff_size = rx_Buff[4];
		calculateChecksum(rx_Buff, Buff_size, &ck_a, &ck_b);

		if (ck_a == rx_Buff[-2] || ck_b == rx_Buff[-1]){
			flag = 1;
		}else{
			flag = 0;
		}

	}else{
		flag = 0;
	}
}

//To use printf
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
