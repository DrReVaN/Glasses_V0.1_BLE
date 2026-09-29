/* CAP1203 driver, original copyright (C) 2021 L. Hummer, GPL-3.0. */
#ifndef STM32_CAP1203_H
#define STM32_CAP1203_H
#include "stm32wbxx_hal.h"
#define CAP1203_I2C_ADDR (0x28u << 1)
HAL_StatusTypeDef CAP1203_Init(I2C_HandleTypeDef *handle);
HAL_StatusTypeDef CAP1203_ReadTouch(uint8_t *pads);
#endif
