#ifndef DISPLAY_TEST_HAL_H
#define DISPLAY_TEST_HAL_H
#include <stddef.h>
#include <stdint.h>
typedef struct { uint8_t unused; } SPI_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
#define GPIOA ((void *)1)
#define GPIO_PIN_2 (1u << 2)
#define GPIO_PIN_3 (1u << 3)
#define GPIO_PIN_4 (1u << 4)
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state);
void HAL_Delay(uint32_t delay);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size, uint32_t timeout);
#endif
