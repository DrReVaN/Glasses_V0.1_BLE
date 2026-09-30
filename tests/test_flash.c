/* Execute the production driver with CPU2/HAL responses, including the
 * PESD-at-BEGIN state measured on the glasses. No physical device is used. */
#include "glasses_ota.h"
#include "glasses_app.h"
#include "app_common.h"
#include "shci.h"
#include "main.h"
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
static bool mode_fail, on_fail, off_fail, pes_after_on, corrupt_write, radio_protection;
static HAL_StatusTypeDef unlock_status, erase_status, write_status, lock_status;
uint32_t glasses_fault[3], glasses_reset_flags;
RTC_HandleTypeDef hrtc;
I2C_HandleTypeDef hi2c1;
static uint32_t backup, resets, jumps;
static unsigned responses;
static unsigned connection_requests;
static uint8_t att_error;
void HAL_PWR_EnableBkUpAccess(void) {}
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h, uint32_t r, uint32_t v) {
    (void)h; assert(r == RTC_BKP_DR6); backup = v;
}
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *h, uint32_t r) {
    (void)h; assert(r == RTC_BKP_DR6); return backup;
}
int CAP1203_Init(I2C_HandleTypeDef *h) { (void)h; return HAL_OK; }
int CAP1203_ReadTouch(uint8_t *pads) { *pads = 0; return HAL_OK; }
void NVIC_SystemReset(void) { ++resets; }
void Glasses_HostJumpApplication(uint32_t sp, uint32_t entry) {
    assert(sp == 0x20007800 && entry == 0x08010141); ++jumps;
}
int aci_gatt_write_resp(uint16_t conn, uint16_t attr, uint8_t status, uint8_t error, uint8_t len, uint8_t *data) {
    (void)conn; (void)attr; (void)status; (void)len; (void)data;
    ++responses; att_error = error; return 0;
}
void Glasses_OtaWrite(uint8_t kind, uint16_t conn, uint16_t attr, const uint8_t *data, uint8_t len);
int aci_l2cap_connection_parameter_update_req(uint16_t conn, uint16_t min, uint16_t max, uint16_t latency, uint16_t timeout) {
    assert(conn == 0x42 && min == 64 && max == 80 && latency == 0 && timeout == 400);
    ++connection_requests; return 0;
}
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
        /* CPU2 takes SEM7 when notified, and only grants an erase window
         * after the next radio event. Reissuing ON re-arms this wait. */
        if (radio_protection) locks[7] = 2;
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
    off_fail = false; locks[2] = 0;
    assert(Glasses_FlashEndErase() == 0);
    memset(locks, 0, sizeof(locks)); memset(releases, 0, sizeof(releases));
    mode_calls = erase_on = erase_off = erases = writes = unlocks = relocks = 0;
    primask = tick = random_calls = 0;
    backup = resets = jumps = responses = att_error = 0;
    connection_requests = 0;
    memset(glasses_fault, 0, sizeof(glasses_fault)); glasses_reset_flags = 0;
    mode_fail = on_fail = off_fail = pes_after_on = corrupt_write = radio_protection = false;
    unlock_status = erase_status = write_status = lock_status = HAL_OK;
    test_flash.SFR = 0x53; test_flash.SR = FLASH_PESD;
    memset((void *)FLASH_BASE, 255, GLASSES_APP_LIMIT - FLASH_BASE);
}
static void ready(void) { reset_fixture(); assert(Glasses_FlashInit()); assert(!test_flash.SR); }
static void clean(unsigned mask) {
    assert(primask == mask && !locks[2] && !locks[7]);
    assert(releases[2] == 1 + erase_off && releases[7] == 1 && relocks == 1);
}
static void drain_ota(void) {
    unsigned start = responses;
    for (unsigned i = 0; i < 20000 && responses == start; ++i) {
        /* CPU2 grant is asynchronous. Production code must return to its
         * foreground loop without restarting the protection handshake. */
        if (tick % 50 == 49 && locks[7] == 2) locks[7] = 0;
        Glasses_OtaProcess(); ++tick;
    }
    assert(responses == start + 1);
}
static void test_receiver_and_real_flash(void) {
    uint8_t image[8205], begin[12] = {'S','G','U','1'}, packet[20];
    uint32_t sp = 0x20007800, entry = 0x08010141, size = sizeof(image);
    memset(image, 0xa5, sizeof(image)); memcpy(image, &sp, 4); memcpy(image + 4, &entry, 4);
    uint32_t crc = glasses_crc32(UINT32_MAX, image, size) ^ UINT32_MAX;
    memcpy(begin + 4, &size, 4); memcpy(begin + 8, &crc, 4);
    ready(); radio_protection = true; Glasses_OtaInit();
    Glasses_OtaPrepareConnection(0x42, 6); assert(connection_requests == 1);
    Glasses_OtaPrepareConnection(0x42, 64); assert(connection_requests == 1);
    Glasses_OtaWrite(1, 1, 1, begin, 12); drain_ota(); assert(!att_error);
    assert(erases == 4 && erase_on == 1 && erase_off == 1); /* metadata and three pages */
    for (uint32_t offset = 0; offset < size; offset += 16) {
        unsigned n = size - offset; if (n > 16) n = 16;
        memcpy(packet, &offset, 4); memcpy(packet + 4, image + offset, n);
        Glasses_OtaWrite(2, 1, 2, packet, n + 4); drain_ota(); assert(!att_error);
    }
    Glasses_OtaWrite(3, 1, 3, (const uint8_t *)"END1", 4); drain_ota(); assert(!att_error);
    const GlassesImage *m = (void *)GLASSES_META_ADDRESS;
    assert(m->magic == GLASSES_IMAGE_MAGIC && m->size == size && m->crc == crc);
    assert(!memcmp((void *)GLASSES_APP_ADDRESS, image, size));
    assert(backup == GLASSES_BOOT_APPLICATION);
    tick += 500; Glasses_OtaProcess(); assert(resets == 1);
    Glasses_BootTryApplication(); assert(jumps == 1 && backup == 0);
    assert(erase_on == 1 && erase_off == 1 && !locks[2] && !locks[7]);
    /* An event callback must defer the cleanup command to foreground. */
    ready(); radio_protection = true; Glasses_OtaInit();
    Glasses_OtaWrite(1, 1, 1, begin, 12); Glasses_OtaProcess(); assert(!erases && erase_on == 1);
    Glasses_OtaDisconnected(); assert(!erase_off);
    locks[2] = 2; Glasses_OtaProcess(); assert(!erase_off);
    locks[2] = 0; Glasses_OtaProcess(); assert(erase_off == 1 && !erases && !writes);
    ready(); radio_protection = true; Glasses_OtaInit();
    Glasses_OtaWrite(1, 1, 1, begin, 12); Glasses_OtaProcess();
    tick += 10001; Glasses_OtaProcess();
    assert(att_error == 0x0e && erase_off == 1 && !erases && !writes);
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
    assert(*(const uint64_t *)GLASSES_META_ADDRESS == UINT64_MAX && erase_on == 1 && erase_off == 0);
    assert(Glasses_FlashWrite(GLASSES_APP_ADDRESS, 0) == 1 && !writes);
    assert(Glasses_FlashEndErase() == 0 && erase_off == 1);
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
    assert(!locks[2] && releases[2] == 1 && !releases[7] && erase_off == 0);
    ready(); locks[7] = 2;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && locks[7] == 2 && !releases[7] && !unlocks);
    assert(!locks[2] && erase_off == 0 && !primask);
    ready(); pes_after_on = true;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && !unlocks && erase_off == 0 && !locks[2]);
    assert(Glasses_FlashEndErase() == 0 && erase_off == 1);
    ready(); on_fail = true;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == -1 && !unlocks && !erase_off && !locks[2]);
    ready(); off_fail = true; locks[6] = 2;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && !unlocks && !locks[2]);
    assert(Glasses_FlashEndErase() == -1 && !locks[2]);
    ready(); off_fail = true;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 0);
    assert(Glasses_FlashEndErase() == -1); clean(0);
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
    ready(); radio_protection = true;
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 1 && !erases);
    assert(erase_on == 1 && erase_off == 0 && !locks[2]);
    locks[7] = 0; /* next radio event: CPU2 grants its erase window */
    assert(Glasses_FlashErase(GLASSES_META_ADDRESS) == 0 && erases == 1);
    assert(erase_on == 1); /* retry must not re-arm CPU2's protection */
    assert(Glasses_FlashErase(GLASSES_APP_ADDRESS) == 0 && erases == 2 && erase_on == 1);
    assert(Glasses_FlashEndErase() == 0 && erase_off == 1);
    test_receiver_and_real_flash();
    puts("Production flash driver + OTA receiver: asynchronous CPU2 erase windows, retries/pages, deferred cancellation, commit/boot, failures, boundaries and retained keys passed.");
    return 0;
}
