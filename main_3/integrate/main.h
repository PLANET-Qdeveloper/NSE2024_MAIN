/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    SAFETY = 0,
    READY,
    BURNING,
    FLIGHT,
    SEP,
    LANDED,
    EMERGENCY
} PHASE;

typedef enum{
	VALVE,
	MAIN1,
	MAIN2,
	MAIN3,
	GROUND
} SEND_TO;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MAIN3_Init();
void measure();
void record();
void send();
void send_GROUND();
void top_detect();
void processGPSData(uint8_t *buffer);
void checkConditionsAndUpdatePhase();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void send_MAIN1(uint8_t *tx_Buff, SEND_TO from, uint8_t com);
void send_VALVE(uint8_t *tx_Buff, uint8_t com);
void send_MAIN2(uint8_t *tx_Buff, SEND_TO want_from, uint8_t com);
void calculateChecksum(uint8_t *Buff, uint8_t Buff_size, uint8_t *ck_a, uint8_t *ck_b);  // for checksum
void checkUART(uint8_t *rx_Buff, uint8_t *ck_a, uint8_t *ck_b);   // if True, it adjusts Buff_size, PHASE


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define SAFETY_PERIOD     1000  // 1Hz
#define READY_PERIOD      1000   // 1Hz
#define BURNING_PERIOD    10   // 100Hz
#define FLIGHT_PERIOD     10   // 100Hz
#define SEP_PERIOD        10   // 100Hz
#define LANDED_PERIOD     10   // 100Hz
#define EMERGENCY_PERIOD  10   // 100Hz
#define MOSFET_PERIOD  3000   // 3s
#define BURNING_TIME  700   // 7s
#define TOP_DETECT_TIME  2000   // 20s
#define SEP_TIME 3 //3s


#define ADDRESS 2024   // inspire from this year

#define TX_BUFF_SIZE_MAIN1 8   // buffer size send to main1
#define TX_BUFF_SIZE_MAIN2 39  // buffer size send to main2

#define RX_BUFF_SIZE_MAIN1 8  // buffer size send to main1
#define RX_BUFF_SIZE_MAIN2 8  // buffer size send to main2

#define GPS_BUFFER 256

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
