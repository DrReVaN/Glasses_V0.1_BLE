#ifndef TEST_FLASH_APP_COMMON_H
#define TEST_FLASH_APP_COMMON_H
#include <stdint.h>
#define FLASH_BASE 0x08000000u
#define FLASH_PAGE_SIZE 4096u
#define FLASH_SFR_SFSA 0xffu
#define FLASH_SFR_SFSA_Pos 0
#define FLASH_FLAG_CFGBSY 0x00040000u
#define FLASH_FLAG_OPTVERR 0x00008000u
#define FLASH_PESD 0x00080000u
#define FLASH_TYPEERASE_PAGES 1u
#define FLASH_TYPEPROGRAM_DOUBLEWORD 2u
#define CFG_HW_FLASH_SEMID 2u
#define CFG_HW_BLOCK_FLASH_REQ_BY_CPU1_SEMID 6u
#define CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID 7u
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { uint32_t SR, SFR; } TestFlash;
typedef struct { uint32_t TypeErase, Page, NbPages; } FLASH_EraseInitTypeDef;
extern TestFlash test_flash;
#define FLASH (&test_flash)
#define HSEM ((void *)0)
int LL_FLASH_IsActiveFlag_OperationSuspended(void);
int LL_HSEM_1StepLock(void *h, unsigned id);
int LL_HSEM_GetStatus(void *h, unsigned id);
void LL_HSEM_ReleaseLock(void *h, unsigned id, unsigned process);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
#define __NOP() ((void)0)
#define __HAL_FLASH_GET_FLAG(flag) (test_flash.SR & (flag))
#define __HAL_FLASH_CLEAR_FLAG(flag) (test_flash.SR &= ~(flag))
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *page, uint32_t *error);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t data);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#endif
