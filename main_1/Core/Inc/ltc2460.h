//
// Created by genta on 2024/05/02.
//

#ifndef POWER_MANAGER_LTC2460_H
#define POWER_MANAGER_LTC2460_H


#include <errno.h>
#include "stm32g0xx_hal.h"
typedef enum {
    HZ_60 = 0,
    HZ_30 = 1
}LTC2460_SPEED;

typedef enum{
    LTC2460_Success = 0,
    LTC2460_NotYetConverted = 1,
    LTC2460_HAL_ERROR = 2
}LTC2460_StatusTypeDef;


typedef struct{
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* ss;
    uint16_t ss_num;
    GPIO_TypeDef* miso;
    uint16_t miso_num;
    uint8_t receive_only;
    LTC2460_SPEED speed;
    uint8_t pTxData[2];
}LTC2460_HandleTypeDef;

LTC2460_StatusTypeDef LTC2460_INIT(LTC2460_HandleTypeDef* ltc2460);
LTC2460_StatusTypeDef LTC2460_SET_SPEED(LTC2460_HandleTypeDef* ltc2460, LTC2460_SPEED speed);
LTC2460_StatusTypeDef LTC2460_READ(LTC2460_HandleTypeDef* ltc2460, uint16_t* value);

#endif //POWER_MANAGER_LTC2460_H
