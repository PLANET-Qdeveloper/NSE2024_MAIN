//
// Created by genta on 2024/05/02.
//
#include <ltc2460.h>



LTC2460_StatusTypeDef LTC2460_INIT(LTC2460_HandleTypeDef *ltc2460) {
    ltc2460->pTxData[0] = 0b1000 << 4; //EN1 = 1 EN2 = 0 SPD = 0:60HZ SLP = 0:Disabled
    ltc2460->pTxData[1] = 0;
    uint8_t pRxData[2];
    HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_RESET);
    if(HAL_SPI_TransmitReceive(ltc2460->spi, ltc2460->pTxData, pRxData, 2, 100) != HAL_OK){
        HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_SET);
        return LTC2460_HAL_ERROR;
    }
    HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_SET);
    return LTC2460_Success;
}

LTC2460_StatusTypeDef LTC2460_SET_SPEED(LTC2460_HandleTypeDef *ltc2460, LTC2460_SPEED speed) {
    ltc2460->pTxData[0] = (0b1000 & speed << 1) << 4;
    ltc2460->pTxData[1] = 0;
    uint8_t pRxData[2];
    HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_RESET);
    if(HAL_SPI_TransmitReceive(ltc2460->spi, ltc2460->pTxData, pRxData, 2, 100) != HAL_OK){
        HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_SET);
        return LTC2460_HAL_ERROR;
    }
    HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_SET);
    return LTC2460_Success;
}

LTC2460_StatusTypeDef LTC2460_READ(LTC2460_HandleTypeDef *ltc2460, uint16_t *value) {
	uint8_t pRxData[2] = {};
    HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_RESET);
    HAL_Delay(1);
    if(!ltc2460->receive_only && HAL_GPIO_ReadPin(ltc2460->miso, ltc2460->miso_num) == GPIO_PIN_SET){
        HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_SET);
        return LTC2460_NotYetConverted;
    }
    HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_RESET);
    HAL_StatusTypeDef stat;
	if(ltc2460->receive_only){
		stat = HAL_SPI_Receive(ltc2460->spi, (uint8_t *)value,2, 100 );
	}else{
		stat = HAL_SPI_TransmitReceive(ltc2460->spi, ltc2460->pTxData, (uint8_t *)value, 2, 100);
	}
	HAL_Delay(1);
    HAL_GPIO_WritePin(ltc2460->ss, ltc2460->ss_num, GPIO_PIN_SET);
    if(stat != HAL_OK){
        return stat + 1;
    }

    HAL_Delay(1);

    return LTC2460_Success;
}
