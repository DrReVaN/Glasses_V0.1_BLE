/* Single-operation CPU1/CPU2 arbitration follows ST AN5289 and the CubeWB
 * v1.13.3 BLE_Ota flash driver. Busy semaphores yield to the foreground.
 * Unlike the reference, HAL errors are propagated and every attempt releases SEM2. */
#include "glasses_ota.h"
#include "glasses_app.h"
#include "app_common.h"
#include "shci.h"
#include "ble.h"
#include <string.h>
static bool flash_ready, erase_active;
bool Glasses_FlashInit(void) {
    flash_ready = SHCI_C2_SetFlashActivityControl(FLASH_ACTIVITY_CONTROL_SEM7) == SHCI_Success;
    return flash_ready;
}
int Glasses_FlashEndErase(void) {
    if (!erase_active) return 0;
    if (LL_HSEM_1StepLock(HSEM, CFG_HW_FLASH_SEMID)) return 1;
    SHCI_CmdStatus_t status = SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_OFF);
    if (status == SHCI_Success) erase_active = false;
    LL_HSEM_ReleaseLock(HSEM, CFG_HW_FLASH_SEMID, 0);
    return status == SHCI_Success ? 0 : -1;
}
static bool writable(uint32_t address) {
    uint32_t sfsa = (FLASH->SFR & FLASH_SFR_SFSA) >> FLASH_SFR_SFSA_Pos;
    return address >= GLASSES_KEYS_ADDRESS && address < GLASSES_APP_LIMIT &&
           address < FLASH_BASE + sfsa * FLASH_PAGE_SIZE;
}
static int operation(uint32_t address, uint64_t data, bool erase) {
    HAL_StatusTypeDef status;
    uint32_t primask, page_error;
    FLASH_EraseInitTypeDef page = {0};
    if (!flash_ready || !writable(address) || (address & (erase ? FLASH_PAGE_SIZE - 1u : 7u))) return -1;
    if (!erase && erase_active) return 1;
    if (LL_FLASH_IsActiveFlag_OperationSuspended() || __HAL_FLASH_GET_FLAG(FLASH_FLAG_CFGBSY)) return 1;
    if (LL_HSEM_1StepLock(HSEM, CFG_HW_FLASH_SEMID)) return 1;
    if (erase && !erase_active) {
        if (SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_ON) != SHCI_Success) {
            LL_HSEM_ReleaseLock(HSEM, CFG_HW_FLASH_SEMID, 0); return -1;
        }
        erase_active = true;
    }
    /* Let CPU2 take SEM7 after erase activity notification (at least 5 us). */
    if (erase) { volatile unsigned i; for (i = 0; i < 70; ++i) __NOP(); }
    primask = __get_PRIMASK(); __disable_irq();
    if (LL_FLASH_IsActiveFlag_OperationSuspended() ||
        LL_HSEM_GetStatus(HSEM, CFG_HW_BLOCK_FLASH_REQ_BY_CPU1_SEMID) ||
        LL_HSEM_1StepLock(HSEM, CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID)) {
        __set_PRIMASK(primask);
        /* CPU2 takes SEM7 until a later radio event. Keep ON across retries:
         * another ON/OFF pair would re-arm protection before we can use it. */
        LL_HSEM_ReleaseLock(HSEM, CFG_HW_FLASH_SEMID, 0); return 1;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    status = HAL_FLASH_Unlock();
    if (status == HAL_OK) {
        if (erase) {
            page.TypeErase = FLASH_TYPEERASE_PAGES; page.Page = (address - FLASH_BASE) / FLASH_PAGE_SIZE; page.NbPages = 1;
            status = HAL_FLASHEx_Erase(&page, &page_error);
        } else status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    }
    /* The watchdog resets into recovery if stalled flash hardware never returns. */
    if (HAL_FLASH_Lock() != HAL_OK) status = HAL_ERROR;
    LL_HSEM_ReleaseLock(HSEM, CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID, 0);
    __set_PRIMASK(primask);
    LL_HSEM_ReleaseLock(HSEM, CFG_HW_FLASH_SEMID, 0);
    if (status != HAL_OK) return -1;
    if (!erase && *(const uint64_t *)address != data) return -1;
    return 0;
}
int Glasses_FlashErase(uint32_t address) { return operation(address, 0, true); }
int Glasses_FlashWrite(uint32_t address, uint64_t data) { return operation(address, data, false); }
/* Generate per-device root keys once and retain them outside the OTA app region.
 * Never derive secret keys from the public chip UID. */
void Glasses_SecurityKeys(uint8_t irk[16], uint8_t erk[16]) {
    const uint32_t *saved = (const uint32_t *)GLASSES_KEYS_ADDRESS;
    uint64_t keys[5];
    uint32_t crc, start;
    unsigned i;
    if (saved[8] == 0x53474B31 &&
        (glasses_crc32(0xFFFFFFFFu, (const uint8_t *)saved, 32) ^ 0xFFFFFFFFu) == saved[9]) {
        memcpy(irk, saved, 16); memcpy(erk, saved + 4, 16); return;
    }
    for (i = 0; i < 4; ++i) if (hci_le_rand((uint8_t *)&keys[i]) != BLE_STATUS_SUCCESS) Glasses_Fatal(7);
    crc = glasses_crc32(0xFFFFFFFFu, (const uint8_t *)keys, 32) ^ 0xFFFFFFFFu;
    keys[4] = ((uint64_t)crc << 32) | 0x53474B31u;
    start = HAL_GetTick();
    for (;;) {
        int result = Glasses_FlashErase(GLASSES_KEYS_ADDRESS);
        if (!result) break;
        if (result < 0 || (uint32_t)(HAL_GetTick() - start) >= 2000) Glasses_Fatal(8);
        HAL_Delay(1); /* One-time provisioning, before advertising/pairing. */
    }
    start = HAL_GetTick();
    for (;;) {
        int result = Glasses_FlashEndErase();
        if (!result) break;
        if (result < 0 || (uint32_t)(HAL_GetTick() - start) >= 2000) Glasses_Fatal(8);
        HAL_Delay(1);
    }
    for (i = 0; i < 5; ++i) {
        start = HAL_GetTick();
        for (;;) {
            int result = Glasses_FlashWrite(GLASSES_KEYS_ADDRESS + 8 * i, keys[i]);
            if (!result) break;
            if (result < 0 || (uint32_t)(HAL_GetTick() - start) >= 2000) Glasses_Fatal(9);
            HAL_Delay(1);
        }
    }
    memcpy(irk, keys, 16); memcpy(erk, keys + 2, 16);
    memset(keys, 0, sizeof(keys));
}
