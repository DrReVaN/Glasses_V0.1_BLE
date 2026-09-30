#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#define FLASH_PAGE_SIZE 4096u
#define RTC_BKP_DR6 6
#define RCC_CSR_IWDGRSTF 0x20000000u
#define HAL_OK 0
#define HAL_ERROR 1
typedef struct { int unused; } RTC_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
uint32_t HAL_GetTick(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h, uint32_t reg, uint32_t value);
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *h, uint32_t reg);
void HAL_Delay(uint32_t ms);
void NVIC_SystemReset(void);
#define __DSB() ((void)0)
#endif
