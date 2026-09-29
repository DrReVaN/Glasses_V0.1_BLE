/* CAP1203 driver, original copyright (C) 2021 L. Hummer, GPL-3.0.
 * Keep the real HAL handle and propagate errors. All accesses are bounded. */
#include "STM32_Cap1203.h"
static I2C_HandleTypeDef *bus;
static HAL_StatusTypeDef write_register(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(bus, CAP1203_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 5);
}
static HAL_StatusTypeDef read_register(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(bus, CAP1203_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 5);
}
HAL_StatusTypeDef CAP1203_Init(I2C_HandleTypeDef *handle) {
    uint8_t id;
    static const uint8_t config[][2] = {
        {0x27, 7}, {0x28, 0}, {0x40, 7}, {0x42, 4}, {0x41, 0x39},
        {0x61, 0}, {0x00, 0x20}
    };
    unsigned i; bus = handle;
    if (!bus || read_register(0xFD, &id) != HAL_OK || id != 0x6D) return HAL_ERROR;
    for (i = 0; i < sizeof(config) / sizeof(config[0]); ++i)
        if (write_register(config[i][0], config[i][1]) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}
HAL_StatusTypeDef CAP1203_ReadTouch(uint8_t *pads) {
    uint8_t status, control;
    if (!bus || !pads || read_register(0x03, &status) != HAL_OK || read_register(0x00, &control) != HAL_OK) return HAL_ERROR;
    if ((control & 1) && write_register(0x00, control & 0xFE) != HAL_OK) return HAL_ERROR;
    *pads = status & 7; return HAL_OK;
}
