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
uint8_t tx_Buff_3[TX_BUFF_SIZE_MAIN1];   // send to 	main1

uint8_t rx_Buff_GPS[1];
uint8_t ReceivedData[GPS_BUFFER];
uint16_t gps_rx_index = 0;

uint8_t command;
uint8_t Buff_size;
uint8_t count;   // adjustment of downlink rate
uint8_t count_pres = 0;

uint8_t ck_a_rx2, ck_b_rx2;   // for checksum
uint8_t ck_a_tx2, ck_b_tx2;   // for checksum
uint8_t ck_a_rx3, ck_b_rx3;   // for checksum
uint8_t ck_a_tx3, ck_b_tx3;   // for checksum

int flag;   // use as bool function. True:1, False:0

uint16_t mission_time = 0;
uint32_t flight_time = 0;
uint32_t sep_time = 0;
uint8_t latitude[2];
uint8_t longitude[2];
uint8_t altitude[3];

uint8_t judg = 0;
uint8_t volt;
uint8_t Vphase = 0;
uint8_t Vpres;

uint8_t RecordData[31];
uint16_t RecordCount = 0;
//uint8_t test[1] = {1};
//uint8_t receivedChar[1];

//uint8_t testBuffer[256];

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
  MAIN3_Init();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  printf("looping\r\n");
	  if (currentPhase != previousPhase) {


		  switch (currentPhase){
		  	  case SAFETY:
				  break;
		  	  case READY:
		  		  HAL_TIM_Base_Start_IT(&htim6);//Flight pin
		  		  MX25L8006E_EraseAll();
				  break;
			  case BURNING:
				  HAL_TIM_Base_Start_IT(&htim15);//Timer1
				  break;
			  case FLIGHT:
				  HAL_TIM_Base_Start_IT(&htim17);//bmp280 top detect
				  break;
			  case SEP:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				  HAL_TIM_Base_Start_IT(&htim14);//Timer2
				  break;
			  case LANDED:
				  break;
			  case EMERGENCY:
				  break;
		  }
	}

	previousPhase = currentPhase;
	switch (currentPhase) {
		  case SAFETY:
			 printf("safety\r\n");
			 measure();
			 HAL_Delay(SAFETY_PERIOD);

			  break;
		  case READY:
			  printf("ready\r\n");
			  measure();
			  HAL_Delay(READY_PERIOD);
			  break;
		  case BURNING:
			  printf("burning\r\n");
			  measure();
			  record();
			  HAL_Delay(BURNING_PERIOD);
			  break;
		  case FLIGHT:
			  //printf("flight\r\n");
			  measure();
			  record();
			  HAL_Delay(FLIGHT_PERIOD);
			  break;
		  case SEP:
			  printf("sep\r\n");
			  measure();
			  record();
			  HAL_Delay(SEP_PERIOD);
			  break;
		  case LANDED:
			  printf("landed\r\n");
			  measure();
			  record();
			  HAL_Delay(LANDED_PERIOD);
			  break;
		  case EMERGENCY:
			  printf("emergency\r\n");
			  measure();
			  record();
			  HAL_Delay(EMERGENCY_PERIOD);
			  break;
	}

//	printf("looping");



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

    HAL_UART_Receive_IT(&huart2, rx_Buff_2, RX_BUFF_SIZE_MAIN2);
    HAL_UART_Receive_IT(&huart3, rx_Buff_3, RX_BUFF_SIZE_MAIN1);
    HAL_UART_Receive_IT(&huart4, rx_Buff_GPS, 1);
//    HAL_UART_Receive_IT(&huart4, testBuffer, 255);
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;

	 const char setRate10Hz[] = "$PMTK220,100*2F\r\n";
	 HAL_UART_Transmit(&huart4, (uint8_t*)setRate10Hz, strlen(setRate10Hz), HAL_MAX_DELAY);
//	 const char setBaudRate[] = "$PMTK251,115200*1F\r\n";
//	 HAL_UART_Transmit(&huart4, (uint8_t*)setBaudRate, strlen(setBaudRate), HAL_MAX_DELAY);
//	 HAL_UART_DeInit(&huart4);
//	     huart4.Init.BaudRate = 115200;
//	     if (HAL_UART_Init(&huart4) != HAL_OK) {
//	         Error_Handler();
//	     }
	 const char setOutput10Hz[] = "$PMTK300,100,0,0,0,0*2C\r\n";
	 HAL_UART_Transmit(&huart4, (uint8_t*)setOutput10Hz, strlen(setOutput10Hz), HAL_MAX_DELAY);
	 const char setGGAOnly[] = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
	 HAL_UART_Transmit(&huart4, (uint8_t*)setGGAOnly, strlen(setGGAOnly), HAL_MAX_DELAY);

	  HAL_SPI_MspInit(&hspi1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // CS=Hにしておく

	    __HAL_SPI_ENABLE(&hspi1);



//	  for (uint8_t i = 0; i < 12; i++) {
//	      uint8_t writeData[31];
//	      memset(writeData, i + 1, sizeof(writeData)); // Fill writeData with the number i+1
//	      MX25L8006E_WriteData(0x000FF1 + i * 31, writeData, 31); // Write data to flash memory
//	  }

//	  readAndPrintData(0x000FF1,12);


	HAL_TIM_Base_Start_IT(&htim16);//Mission time
	HAL_TIM_Base_Start_IT(&htim7);//Send ground


	while (!bmp280_init(&bmp280, &bmp280.params))
	{
		printf("BME280 initialization failed\r\n");
		HAL_Delay(2000);
	}
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
	if(RecordCount < 33825){
		int temp_decimal_first_digit = (int)(temperature * 10) % 10;
		uint8_t flight_pin_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

		uint8_t accx_int = (int)accelerometer.x;
		uint8_t accx_decimal = (int)((accelerometer.x - accx_int) * 100);
		RecordData[16] = accx_int & 0xFF;
		RecordData[17] = accx_decimal & 0xFF;

		uint8_t accy_int = (int)accelerometer.y;
		uint8_t accy_decimal = (int)((accelerometer.y - accy_int) * 100);
		RecordData[18] = accy_int & 0xFF;
		RecordData[19] = accy_decimal & 0xFF;

		uint8_t accz_int = (int)accelerometer.z;
		uint8_t accz_decimal = (int)((accelerometer.z - accz_int) * 100);
		RecordData[20] = accz_int & 0xFF;
		RecordData[21] = accz_decimal & 0xFF;

		uint8_t gyrx_int = (int)gyroscope.x;
		uint8_t gyrx_decimal = (int)((gyroscope.x - gyrx_int) * 100);
		RecordData[22] = gyrx_int & 0xFF;
		RecordData[23] = gyrx_decimal & 0xFF;

		uint8_t gyry_int = (int)gyroscope.y;
		uint8_t gyry_decimal = (int)((gyroscope.y - gyry_int) * 100);
		RecordData[24] = gyry_int & 0xFF;
		RecordData[25] = gyry_decimal & 0xFF;

		uint8_t gyrz_int = (int)gyroscope.z;
		uint8_t gyrz_decimal = (int)((gyroscope.z - gyrz_int) * 100);
		RecordData[26] = gyrz_int & 0xFF;
		RecordData[27] = gyrz_decimal & 0xFF;


		RecordData[0] = currentPhase;
		RecordData[1] = (flight_time >> 8) & 0xFF;
		RecordData[2] = flight_time & 0xFF;
		RecordData[3] = latitude[0];
		RecordData[4] = latitude[1];
		RecordData[5] = longitude[0];
		RecordData[6] = longitude[1];
		RecordData[7] = altitude[0];
		RecordData[8] = altitude[1];
		RecordData[9] = altitude[2];
		RecordData[10] = ((int)pressure >> 16) & 0xFF;
		RecordData[11] = ((int)pressure >> 8) & 0xFF;
		RecordData[12] = (int)pressure & 0xFF;
		RecordData[13] =(int)temperature;
		RecordData[14] = ((temp_decimal_first_digit & 0x0F) << 4) | (judg << 2) | (flight_pin_status << 1);
		RecordData[15] = volt;
		RecordData[28] = Vphase;
		RecordData[29] = Vpres;
		RecordData[30] = 0b11111111;
		MX25L8006E_WriteData(0x000000 + RecordCount * 31, RecordData, 31); // Write data to flash memory
		RecordCount ++;
//		if(RecordCount % 10 == 0){
//			readAndPrintData(0x000000 + (RecordCount - 10) * 31 ,10);
//		}
	}

}

void top_detect(){
	if(previous_pressure <= pressure){
		count_pres += 1;
		printf("count : %d\r\n", count_pres);
		if(count_pres >= 10){
		    HAL_TIM_Base_Stop_IT(&htim17);
			currentPhase = SEP;
			judg = 2;
			count_pres = 0;
		}
	}else{
		count_pres = 0;
	}
	previous_pressure = pressure;
}

void processGPSData(uint8_t *buffer){
    char *ggaMessage = strstr((char *)buffer, "$GPGGA");
    if (ggaMessage == NULL) {
        return;
    }

    char* rest = ggaMessage + 7;  // Skip "$GPGGA,"
    char token[20];
    int idx = 0;

    // Skip time
    while (*rest != ',' && *rest != '\0') rest++;
    if (*rest == ',') rest++;

    // Get latitude
    idx = 0;
    while (*rest != ',' && *rest != '\0' && idx < sizeof(token) - 1) {
        token[idx++] = *rest++;
    }
    token[idx] = '\0';
    if (*rest == ',') rest++;

    if (idx > 0) {
        uint8_t latitude_int = (token[0] - '0') * 10 + (token[1] - '0');
        if (latitude_int == 0) {
            latitude[0] = 0x55;
            latitude[1] = 0x55;
        } else {
            latitude[0] = (token[2] - '0') * 10 + (token[3] - '0');
            latitude[1] = (token[5] - '0') * 10 + (token[6] - '0');
        }
    } else {
        latitude[0] = 0x55;
        latitude[1] = 0x55;
    }

    // Skip N/S indicator
    while (*rest != ',' && *rest != '\0') rest++;
    if (*rest == ',') rest++;

    // Get longitude
    idx = 0;
    while (*rest != ',' && *rest != '\0' && idx < sizeof(token) - 1) {
        token[idx++] = *rest++;
    }
    token[idx] = '\0';
    if (*rest == ',') rest++;

    if (idx > 0) {
        uint8_t longitude_int = (token[0] - '0') * 100 + (token[1] - '0') * 10 + (token[2] - '0');
        if (longitude_int == 0) {
            longitude[0] = 0x55;
            longitude[1] = 0x55;
        } else {
            longitude[0] = (token[3] - '0') * 10 + (token[4] - '0');
            longitude[1] = (token[6] - '0') * 10 + (token[7] - '0');
        }
    } else {
        longitude[0] = 0x55;
        longitude[1] = 0x55;
    }

    // Skip E/W indicator
    while (*rest != ',' && *rest != '\0') rest++;
    if (*rest == ',') rest++;

    while (*rest != ',' && *rest != '\0') rest++;
	if (*rest == ',') rest++;

    // Skip number of satellites
    while (*rest != ',' && *rest != '\0') rest++;
    if (*rest == ',') rest++;

    while (*rest != ',' && *rest != '\0') rest++;
    if (*rest == ',') rest++;

    // Get altitude
    idx = 0;
    while (*rest != ',' && *rest != '\0' && idx < sizeof(token) - 1) {
        token[idx++] = *rest++;
    }
    token[idx] = '\0';

    if (idx > 0) {
        double altitude_double = atof(token);
        uint16_t altitude_int = (int)altitude_double;
        uint8_t altitude_decimal = (int)((altitude_double - altitude_int) * 100);
        altitude[0] = (altitude_int >> 8) & 0xFF;
        altitude[1] = altitude_int & 0xFF;
        altitude[2] = altitude_decimal & 0xFF;
    } else {
        altitude[0] = 0x00;
        altitude[1] = 0x00;
        altitude[2] = 0x00;
    }
}

//Timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {//Flight pin
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET){
		  HAL_TIM_Base_Stop_IT(&htim6);
		  currentPhase = BURNING;
	  }
    }else if (htim->Instance == TIM7) {
    	send_GROUND();
	}else if (htim->Instance == TIM14) {
		sep_time += 1;
		if(currentPhase == SEP){
			if(sep_time >= SEP_TIME){
			HAL_TIM_Base_Stop_IT(&htim14);
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			currentPhase = LANDED;
			}
		}
	}else if (htim->Instance == TIM15) {
		printf("count up \r\n");
		flight_time += 1;
		if(currentPhase == BURNING){
			if(flight_time >= BURNING_TIME){
				currentPhase = FLIGHT;
			}
		}else if(currentPhase == FLIGHT){
			if(flight_time >= TOP_DETECT_TIME){
				currentPhase = SEP;
				judg = 1;
			}
		}
    }else if (htim->Instance == TIM16) {
		mission_time += 1;
//		switch (currentPhase) {
//				  	  case SAFETY:
//				  		if(mission_time >= 10){
//				  			currentPhase = READY;
//				  		}
//						  break;
//				  	  case READY:
//				  		if(mission_time >= 20){
//							currentPhase = BURNING;
//						}
//						  break;
//					  case BURNING:
//						  if(mission_time >= 30){
//							currentPhase = FLIGHT;
//						}
//						  break;
//					  case FLIGHT:
//						  if(mission_time >= 40){
//								currentPhase = SEP;
//							}
//						  break;
//					  case SEP:
//						  if(mission_time >= 50){
//							currentPhase = LANDED;
//						}
//					  case LANDED:
//						  if(mission_time >= 60){
//							currentPhase = EMERGENCY;
//						}
//						  break;
//					  case EMERGENCY:
//
//						  break;
//	    }
	}else if (htim->Instance == TIM17) {
    	if(currentPhase == FLIGHT){
    		top_detect();
    	}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) { // use ck_rx2
		printf("receive");
//			checkUART(rx_Buff_2, &ck_a_rx2, &ck_b_rx2);
		flag = 1;

			if (flag == 1){
	            command = rx_Buff_2[4];

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
	                                judg = 3;
	                                break;

	                            case LANDED:
	                                currentPhase = EMERGENCY;
	                                break;

	                            case EMERGENCY:
	                                currentPhase = SAFETY;
	                                if(flight_time > 0){
	                                HAL_TIM_Base_Stop_IT(&htim15);
	                                flight_time = 0;
	                                }
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
	            	printf("valve!!\r\n");
	                send_VALVE(tx_Buff_3, command);

	            }else if (rx_Buff_2[3] == MAIN1){
	                sendfrom = MAIN2;

//	                send_MAIN1(tx_Buff_3, sendfrom, command);

	            }


			}

			flag = 0;
			MX_USART2_UART_Init();
			memset(rx_Buff_2, 0, RX_BUFF_SIZE_MAIN2);
			HAL_UART_Receive_IT(&huart2, rx_Buff_2, RX_BUFF_SIZE_MAIN2);
		}

	    if (huart->Instance == USART3) {  // use ck_rx3
	    	printf("recive from main1\r\n");
			checkUART(rx_Buff_3, &ck_a_rx3, &ck_b_rx3);
			volt = rx_Buff_3[5];
			Vphase = rx_Buff_3[6];
			Vpres = rx_Buff_3[7];
			printf("%d,%d,%d\r\n", volt, Vphase, Vpres);

	        if (flag == 1){
	        	if (rx_Buff_3[3] == MAIN1){
	        		volt = rx_Buff_3[5];
	        		Vphase = rx_Buff_3[6];
	        		Vpres = rx_Buff_3[7];
	        		printf("%d,%d,%d\r\n", volt, Vphase, Vpres);
	        	}
//	            if (rx_Buff_3[3] == MAIN1){
//	                sendfrom = MAIN3;
//	                command = rx_Buff_3[5];
//
//	                send_MAIN1(tx_Buff_3, sendfrom, command);
//
//	            }else if (rx_Buff_2[3] == MAIN2){
//	                sendfrom = MAIN1;
//	                command = rx_Buff_3[5];
//
//	                send_MAIN2(tx_Buff_2, sendfrom, command);
//	            }
	        }

	        flag = 0;
			MX_USART3_UART_Init();
			memset(rx_Buff_3, 0, RX_BUFF_SIZE_MAIN1);
			HAL_UART_Receive_IT(&huart3, rx_Buff_3, RX_BUFF_SIZE_MAIN1);

	    }
		if (huart->Instance == USART4) {
			// 変数の宣�?
			char temporalData[1];
			temporalData[0] = (char)rx_Buff_GPS[0];
			ReceivedData[gps_rx_index] = temporalData[0];

			if (temporalData[0] == '\n') { // 受信�?ータの終端を判�?
//				printf("end of sentence\r\n");
				ReceivedData[GPS_BUFFER - 1] = '\0';
				processGPSData(ReceivedData);
				memset(ReceivedData, 0, GPS_BUFFER);
//				char charData[GPS_BUFFER];
//				strncpy(charData, (char *)ReceivedData, GPS_BUFFER);
//				charData[GPS_BUFFER - 1] = '\0'; // �?字�?��?�終端を設�?
//				printf("%s", charData); // printfの形式指定子を使用
				// HAL_UART_Transmit(&huart1, (uint8_t *)charData, GPS_BUFFER, HAL_MAX_DELAY);
//				memset(charData, 0, GPS_BUFFER);
				gps_rx_index = 0;
			} else {
				gps_rx_index++;
				if (gps_rx_index >= GPS_BUFFER) {
					gps_rx_index = 0; // バッファオーバ�?�フロー防止のためリセ�?�?
				}
			}

			// 次のバイトを受信
			HAL_UART_Receive_IT(huart, rx_Buff_GPS, 1);
		}




//	if (huart->Instance == USART4) {
////		printf("receive from GPS\r\n");
////		printf("%d\r\n",gps_rx_index);
//
////		testBuffer[255] = '\0';
////		char buffer[256];
////		strncpy(buffer, (char *)testBuffer, sizeof(buffer));
////		printf(buffer);
////		MX_USART4_UART_Init();
////		 HAL_UART_Receive_IT(&huart4, testBuffer, 255);
//
//		char temporalData[1];
//		ReceivedData[gps_rx_index] = rx_Buff_GPS[0];
//		temporalData[0] = (char)rx_Buff_GPS[0];
//
//        if (temporalData[0] == '\n') { // 受信�?ータの終端を判�?
//        	printf("end of sentence\r\n");
//        	char charData[GPS_BUFFER];
//        	strncpy(charData, (char *)ReceivedData, GPS_BUFFER);
//        	charData[GPS_BUFFER - 1] = '\0'; // �?字�?��?�終端を設�?
//        	printf(charData);
////        	HAL_UART_Transmit(&huart1, (uint8_t *)charData, GPS_BUFFER,HAL_MAX_DELAY);
//            memset(charData,0,GPS_BUFFER);
//            gps_rx_index = 0;
//        } else {
//        	gps_rx_index ++;
//			if (gps_rx_index >= GPS_BUFFER) {
//				gps_rx_index = 0; // バッファオーバ�?�フロー防止のためリセ�?�?
//			}
//        }
////        HAL_UART_Receive_IT(huart, &rx_Buff_GPS[gps_rx_index], 1); // 次のバイトを受信
////	    rx_Buff_GPS[GPS_BUFFER - 1] = '\0';
//
////	    processGPSData(rx_Buff_GPS);
////	    memset(rx_Buff_GPS,0,GPS_BUFFER);
//	    MX_USART4_UART_Init();
//       	HAL_UART_Receive_IT(huart, rx_Buff_GPS, 1);
//
//       }
}


void send_MAIN2(uint8_t *tx_Buff, SEND_TO want_from, uint8_t com){ // use ck_tx2
	sendto = MAIN2;

	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[3] = want_from;
	tx_Buff[4] = TX_BUFF_SIZE_MAIN2;

    if (want_from == MAIN1){
        tx_Buff[5] = com;

    }


	calculateChecksum(tx_Buff, TX_BUFF_SIZE_MAIN2, &ck_a_tx2, &ck_b_rx2);
	tx_Buff[-2] = ck_a_rx2;
	tx_Buff[-1] = ck_b_rx2;

}

void send_GROUND(){
	int temp_decimal_first_digit = (int)(temperature * 10) % 10;
	uint8_t flight_pin_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

	uint8_t accx_int = (int)accelerometer.x;
	uint8_t accx_decimal = (int)((accelerometer.x - accx_int) * 100);
	tx_Buff_2[23] = accx_int & 0xFF;
	tx_Buff_2[24] = accx_decimal & 0xFF;

	uint8_t accy_int = (int)accelerometer.y;
	uint8_t accy_decimal = (int)((accelerometer.y - accy_int) * 100);
	tx_Buff_2[25] = accy_int & 0xFF;
	tx_Buff_2[26] = accy_decimal & 0xFF;

	uint8_t accz_int = (int)accelerometer.z;
	uint8_t accz_decimal = (int)((accelerometer.z - accz_int) * 100);
	tx_Buff_2[27] = accz_int & 0xFF;
	tx_Buff_2[28] = accz_decimal & 0xFF;

	uint8_t gyrx_int = (int)gyroscope.x;
	uint8_t gyrx_decimal = (int)((gyroscope.x - gyrx_int) * 100);
	tx_Buff_2[29] = gyrx_int & 0xFF;
	tx_Buff_2[30] = gyrx_decimal & 0xFF;

	uint8_t gyry_int = (int)gyroscope.y;
	uint8_t gyry_decimal = (int)((gyroscope.y - gyry_int) * 100);
	tx_Buff_2[31] = gyry_int & 0xFF;
	tx_Buff_2[32] = gyry_decimal & 0xFF;

	uint8_t gyrz_int = (int)gyroscope.z;
	uint8_t gyrz_decimal = (int)((gyroscope.z - gyrz_int) * 100);
	tx_Buff_2[33] = gyrz_int & 0xFF;
	tx_Buff_2[34] = gyrz_decimal & 0xFF;


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
	tx_Buff_2[21] = ((temp_decimal_first_digit & 0x0F) << 4) | (judg << 2) | (flight_pin_status << 1);
	tx_Buff_2[22] = volt;
	tx_Buff_2[35] = Vphase;
	tx_Buff_2[36] = Vpres;

	calculateChecksum(tx_Buff_2, TX_BUFF_SIZE_MAIN2, &ck_a_tx2, &ck_b_tx2);
	tx_Buff_2[TX_BUFF_SIZE_MAIN2 - 2] = ck_a_tx2;
	tx_Buff_2[TX_BUFF_SIZE_MAIN2 - 1] = ck_b_tx2;

	HAL_UART_Transmit(&huart2, tx_Buff_2, TX_BUFF_SIZE_MAIN2, HAL_MAX_DELAY);

}


void send_VALVE(uint8_t *tx_Buff, uint8_t com){ // use ck_tx3
	sendto = MAIN1;
	wantto = VALVE;

	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
	tx_Buff[1] = ADDRESS & 0xFF;
	tx_Buff[2] = sendto;
	tx_Buff[3] = wantto;
	tx_Buff[4] = TX_BUFF_SIZE_MAIN1;
    tx_Buff[5] = com;

	calculateChecksum(tx_Buff, TX_BUFF_SIZE_MAIN1, &ck_a_tx3, &ck_b_tx3);
	tx_Buff[TX_BUFF_SIZE_MAIN1 -2] = ck_a_tx3;
	tx_Buff[TX_BUFF_SIZE_MAIN1 -1] = ck_b_tx3;
	HAL_UART_Transmit(&huart3, tx_Buff_3, TX_BUFF_SIZE_MAIN1, HAL_MAX_DELAY);

}


//void send_MAIN1(uint8_t *tx_Buff, SEND_TO from, uint8_t com){  // use ck_tx3
//	sendto = MAIN1;
//
//	tx_Buff[0] = (ADDRESS >> 8) & 0xFF;
//	tx_Buff[1] = ADDRESS & 0xFF;
//	tx_Buff[2] = sendto;
//	tx_Buff[3] = from;
//	tx_Buff[4] = TX_BUFF_SIZE_MAIN1;
//    tx_Buff[5] = com;
//
//	calculateChecksum(tx_Buff, TX_BUFF_SIZE_MAIN1, &ck_a_tx3, &ck_b_tx3);
//	tx_Buff[-2] = ck_a_tx3;
//	tx_Buff[-1] = ck_b_tx3;
//
//}


void calculateChecksum(uint8_t *Buff, uint8_t Buff_size, uint8_t *ck_a, uint8_t *ck_b){
	*ck_a = 0;
    *ck_b = 0;

    for (int i = 0; i < (Buff_size - 2); i++) {
        *ck_a = (*ck_a + Buff[i]) & 0xFF;
        *ck_b = (*ck_b + *ck_a) & 0xFF;
    }
//	printf("ck_a : %d\r\n", *ck_a);
//	printf("ck_b : %d\r\n", *ck_b);
}


void checkUART(uint8_t *rx_Buff, uint8_t *ck_a, uint8_t *ck_b){
	if (((rx_Buff[0] << 8) | (rx_Buff[1])) == ADDRESS || rx_Buff[2] == 2){
        Buff_size = rx_Buff[4];
		calculateChecksum(rx_Buff, Buff_size, ck_a, ck_b);

		if (*ck_a == rx_Buff[-2] || *ck_b == rx_Buff[-1]){
			flag = 1;
		}else{
			flag = 0;
		}

	}else{
		flag = 0;
	}
}

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
          HAL_Delay(1);
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

  void MX25L8006E_ReadData(uint32_t address, uint8_t *buffer, uint32_t length) {
      uint8_t cmd[4];

	  cmd[0] = MX25L8006E_READ_DATA;
	  cmd[1] = (address >> 16) & 0xFF;
	  cmd[2] = (address >> 8) & 0xFF;
	  cmd[3] = address & 0xFF;

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // CS Low
	  HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
	  HAL_SPI_Receive(&hspi1, buffer, length, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // CS High


  }

  void readAndPrintData(uint32_t start_address, uint32_t num_blocks) {
      uint8_t buffer[31];
      for (uint32_t i = 0; i < num_blocks; i++) {
          uint32_t address = start_address + (i * 31);
          MX25L8006E_ReadData(address, buffer, 31);
          HAL_UART_Transmit(&huart1, (uint8_t*)"Read Data: ", strlen("Read Data: "), HAL_MAX_DELAY);
          printHex(buffer, 31);
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

//To use printf
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,HAL_MAX_DELAY);
  return len;
}
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
