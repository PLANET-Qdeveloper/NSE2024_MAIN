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

void printHex(uint8_t *data, uint16_t length) ;
void MX25L8006E_SectorErase(uint32_t address);
void MX25L8006E_WriteEnable(void);
void MX25L8006E_WaitForWriteComplete(void);
void MX25L8006E_PageProgram(uint32_t address, uint8_t *data, uint16_t length);
void MX25L8006E_WriteData(uint32_t start_address, uint8_t *data, uint32_t total_length);
void MX25L8006E_ReadData(uint32_t start_address, uint8_t *buffer, uint32_t length);
void readAndPrintData(uint32_t start_address, uint32_t num_blocks);
void MX25L8006E_EraseAll(void);


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
#define BURNING_TIME  500   // 5s
#define TOP_DETECT_TIME  1500  // 15s
#define SEP_TIME 5 //5s


#define ADDRESS 2024   // inspire from this year

#define TX_BUFF_SIZE_MAIN1 8   // buffer size send to main1
#define TX_BUFF_SIZE_MAIN2 39  // buffer size send to main2

#define RX_BUFF_SIZE_MAIN1 8  // buffer size send to main1
#define RX_BUFF_SIZE_MAIN2 8  // buffer size send to main2

#define GPS_BUFFER 256

#define MX25L8006E_READ_DATA 0x03
#define MX25L8006E_WRITE_ENABLE 0x06
#define MX25L8006E_PAGE_PROGRAM 0x02
#define MX25L8006E_SECTOR_ERASE 0x20
#define MX25L8006E_READ_STATUS 0x05
#define MX25L8006E_READ_ID 0x9F

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
