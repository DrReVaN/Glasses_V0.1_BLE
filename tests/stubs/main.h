#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#define FLASH_PAGE_SIZE 4096u
#define RTC_BKP_DR6 6
typedef struct { int unused; } RTC_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
uint32_t HAL_GetTick(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h, uint32_t reg, uint32_t value);
void NVIC_SystemReset(void);
#define __DSB() ((void)0)
#endif
