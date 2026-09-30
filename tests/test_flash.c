/* Execute the production driver with CPU2/HAL responses, including the
 * PESD-at-BEGIN state measured on the glasses. No physical device is used. */
#include "glasses_ota.h"
#include "glasses_app.h"
#include "app_common.h"
#include "shci.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif
TestFlash test_flash;
static unsigned locks[8], releases[8], mode_calls, erase_on, erase_off;
static unsigned erases, writes, unlocks, relocks, random_calls;
static uint32_t primask, tick;
static bool mode_fail, on_fail, off_fail, pes_after_on, corrupt_write;
static HAL_StatusTypeDef unlock_status, erase_status, write_status, lock_status;
SHCI_CmdStatus_t SHCI_C2_SetFlashActivityControl(SHCI_C2_SET_FLASH_ACTIVITY_CONTROL_Source_t source) {
    assert(source == FLASH_ACTIVITY_CONTROL_SEM7 && !locks[2] && !locks[7]);
    ++mode_calls;
    if (mode_fail) return SHCI_Error;
    test_flash.SR &= ~FLASH_PESD; /* CPU2 changes from its default PES protection. */
    return SHCI_Success;
}
SHCI_CmdStatus_t SHCI_C2_FLASH_EraseActivity(SHCI_EraseActivity_t activity) {
    assert(mode_calls && locks[2] == 1 && locks[7] != 1);
    if (activity == ERASE_ACTIVITY_ON) {
        ++erase_on;
        if (pes_after_on) test_flash.SR |= FLASH_PESD;
        return on_fail ? SHCI_Error : SHCI_Success;
    }
    ++erase_off; return off_fail ? SHCI_Error : SHCI_Success;
}
int LL_FLASH_IsActiveFlag_OperationSuspended(void) { return !!(test_flash.SR & FLASH_PESD); }
int LL_HSEM_1StepLock(void *h, unsigned id) {
    (void)h; assert(id == 2 || id == 7);
    if (locks[id]) return 1;
    locks[id] = 1; return 0;
}
int LL_HSEM_GetStatus(void *h, unsigned id) { (void)h; assert(id == 6); return locks[id]; }
void LL_HSEM_ReleaseLock(void *h, unsigned id, unsigned process) {
    (void)h; assert(process == 0 && locks[id] == 1);
    ++releases[id]; locks[id] = 0;
}
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1; }
void __set_PRIMASK(uint32_t value) { primask = value; }
static void hal_guard(void) { assert(mode_calls && locks[2] == 1 && locks[7] == 1 && primask == 1); }
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { hal_guard(); ++unlocks; return unlock_status; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { hal_guard(); ++relocks; return lock_status; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *page, uint32_t *error) {
    hal_guard(); assert(page->TypeErase == FLASH_TYPEERASE_PAGES && page->NbPages == 1 && erase_on);
    ++erases; *error = 0;
    if (erase_status == HAL_OK) memset((void *)(uintptr_t)(FLASH_BASE + page->Page * FLASH_PAGE_SIZE), 255, FLASH_PAGE_SIZE);
    return erase_status;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t data) {
    hal_guard(); assert(type == FLASH_TYPEPROGRAM_DOUBLEWORD && !(address & 7)); ++writes;
    if (write_status == HAL_OK && !corrupt_write) {
        uint8_t *p = (void *)(uintptr_t)address, *d = (void *)&data;
        for (unsigned i = 0; i < 8; ++i) { assert((p[i] & d[i]) == d[i]); p[i] = d[i]; }
    }
    return write_status;
}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
int hci_le_rand(uint8_t *out) { memset(out, ++random_calls, 8); return 0; }
void Glasses_Fatal(uint32_t code) { (void)code; assert(!"Unexpected fatal"); }
static void reset_fixture(void) {
    memset(locks, 0, sizeof(locks)); memset(releases, 0, sizeof(releases));
    mode_calls = erase_on = erase_off = erases = writes = unlocks = relocks = 0;
    primask = tick = random_calls = 0;
    mode_fail = on_fail = off_fail = pes_after_on = corrupt_write = false;
    unlock_status = erase_status = write_status = lock_status = HAL_OK;
    test_flash.SFR = 0x53; test_flash.SR = FLASH_PESD;
    memset((void *)FLASH_BASE, 255, GLASSES_APP_LIMIT - FLASH_BASE);
}
static void ready(void) { reset_fixture(); assert(Glasses_FlashInit()); assert(!test_flash.SR); }
static void clean(unsigned mask) {
    assert(primask == mask && !locks[2] && !locks[7]);
    assert(releases[2] == 1 && releases[7] == 1 && relocks == 1);
}
int main(void) {
#ifdef _WIN32
    assert(VirtualAlloc((void *)FLASH_BASE, GLASSES_APP_LIMIT - FLASH_BASE,
                        MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE) == (void *)FLASH_BASE);
#else
    assert(mmap((void *)FLASH_BASE, GLASSES_APP_LIMIT - FLASH_BASE, PROT_READ | PROT_WRITE,
                MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)FLASH_BASE);
#endif
    reset_fixture();
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == -1 && !unlocks);
    mode_fail = true; assert(!Glasses_FlashInit());
    assert(Glasses_FlashWrite(GLASSES_APP_ADDRESS, 0) == -1 && !unlocks);
    /* Reproduce the live failure: PESD is set and metadata still valid. Only
     * selecting CPU2 SEM7 permits the receiver's first metadata erase. */
    mode_fail = false;
    memset((void *)GLASSES_META_ADDRESS, 0x5a, FLASH_PAGE_SIZE);
    assert(test_flash.SR == FLASH_PESD && Glasses_FlashInit());
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 0 && erases == 1);
    assert(*(const uint64_t *)GLASSES_META_ADDRESS == UINT64_MAX && erase_on == 1 && erase_off == 1);
    clean(0);
    ready();
    assert(Glasses_FlashWrite(GLASSES_APP_ADDRESS, 0x123456789abcdef0ULL) == 0 && writes == 1);
    assert(*(const uint64_t *)GLASSES_APP_ADDRESS == 0x123456789abcdef0ULL); clean(0);
    ready(); primask = 1; assert(Glasses_FlashErase(GLASSES_APP_ADDRESS) == 0); clean(1);
    ready(); test_flash.SR = FLASH_PESD;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && !erase_on && !unlocks);
    ready(); test_flash.SR = FLASH_FLAG_CFGBSY;
    assert(Glasses_FlashWrite(GLASSES_APP_ADDRESS, 0) == 1 && !unlocks);
    ready(); locks[2] = 2;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && locks[2] == 2 && !releases[2]);
    ready(); locks[6] = 2;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && locks[6] == 2 && !unlocks && !primask);
    assert(!locks[2] && releases[2] == 1 && !releases[7] && erase_off == 1);
    ready(); locks[7] = 2;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && locks[7] == 2 && !releases[7] && !unlocks);
    assert(!locks[2] && erase_off == 1 && !primask);
    ready(); pes_after_on = true;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && !unlocks && erase_off == 1 && !locks[2]);
    ready(); on_fail = true;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == -1 && !unlocks && !erase_off && !locks[2]);
    ready(); off_fail = true; locks[6] = 2;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == -1 && !unlocks && !locks[2]);
    ready(); off_fail = true;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == -1); clean(0);
    ready(); unlock_status = HAL_ERROR;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == -1 && !erases); clean(0);
    ready(); erase_status = HAL_ERROR;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == -1); clean(0);
    ready(); write_status = HAL_ERROR;
    assert(Glasses_FlashWrite(GLASSES_APP_ADDRESS, 0) == -1); clean(0);
    ready(); lock_status = HAL_ERROR;
    assert(Glasses_FlashWrite(GLASSES_APP_ADDRESS, 0) == -1); clean(0);
    ready(); corrupt_write = true;
    assert(Glasses_FlashWrite(GLASSES_APP_ADDRESS, 0) == -1); clean(0);
    ready();
    assert(Glasses_FlashErase(FLASH_BASE) == -1);
    assert(Glasses_FlashErase(GLASSES_KEYS_ADDRESS - FLASH_PAGE_SIZE) == -1);
    assert(Glasses_FlashErase(GLASSES_APP_LIMIT) == -1);
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS + 1) == -1);
    assert(Glasses_FlashWrite(GLASSES_APP_ADDRESS + 1, 0) == -1 && !unlocks);
    test_flash.SFR = (GLASSES_APP_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
    assert(Glasses_FlashErase(GLASSES_APP_ADDRESS) == -1 && !unlocks);
    ready();
    uint8_t irk[16], erk[16], saved[32];
    Glasses_SecurityKeys(irk, erk); assert(erases == 1 && writes == 5 && random_calls == 4);
    memcpy(saved, irk, 16); memcpy(saved + 16, erk, 16);
    Glasses_SecurityKeys(irk, erk); assert(erases == 1 && writes == 5 && random_calls == 4);
    assert(!memcmp(saved, irk, 16) && !memcmp(saved + 16, erk, 16));
    assert(primask == 0 && !locks[2] && !locks[7]);
    puts("Production flash driver: CPU2 mode, contention, failures, boundaries and retained keys passed.");
    return 0;
}
