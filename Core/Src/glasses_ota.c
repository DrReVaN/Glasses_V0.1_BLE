#include "glasses_ota.h"
#include "glasses_app.h"
#include "main.h"
#include "ble.h"
#include "STM32_Cap1203.h"
#include <string.h>
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern uint32_t glasses_fault[3], glasses_reset_flags;
static bool boot_request_write(uint32_t request) {
    /* Flush the APB-AHB bridge before accessing the backup domain, as in
     * CubeWB Reset_BackupDomain. Read back the request before any reset. */
    HAL_PWR_EnableBkUpAccess();
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6, request);
    return HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6) == request;
}
void Glasses_OtaReboot(void) {
    if (!boot_request_write(GLASSES_BOOT_REQUEST)) return;
    __DSB(); NVIC_SystemReset();
}
#ifdef SMARTGLASSES_BOOTLOADER
void Glasses_OtaPrepareConnection(uint16_t conn, uint16_t interval) {
    /* CPU2 permits an erase only with at least 25 ms of RF idle. A short
     * phone-selected interval can otherwise keep SEM7 locked indefinitely.
     * Request 80-100 ms once when connecting; the phone negotiates the result. */
    if (interval < 64) (void)aci_l2cap_connection_parameter_update_req(conn, 64, 80, 0, 400);
}
static uint32_t expected_size, expected_crc, received, erase_page, verify_at, crc, last;
static uint32_t reboot_at;
static uint16_t pending_conn, pending_attr;
static uint8_t pending[20], pending_len, pending_kind;
static uint8_t word_used, packet_at, metadata_word;
static uint64_t word;
static bool uploading, request_pending, commit, erase_manifest, rebooting;
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void reply(uint8_t error) {
    aci_gatt_write_resp(pending_conn, pending_attr, error != 0, error,
                        error ? 0 : pending_len, pending);
    request_pending = false;
    if (error) uploading = commit = false;
}
void Glasses_OtaDisconnected(void) { uploading = commit = request_pending = false; }
void Glasses_OtaInit(void) { Glasses_OtaDisconnected(); rebooting = false; }
/* Called only for authenticated encrypted ATT Write Requests.
 * No flash writes occur in the BLE event handler. One outstanding request is held. */
void Glasses_OtaWrite(uint8_t kind, uint16_t conn, uint16_t attr, const uint8_t *data, uint8_t len) {
    uint8_t error = 0;
    if (request_pending || rebooting) error = 0x09;
    else if (!data || len > sizeof(pending)) error = 0x0D;
    else if (kind == 1) {
        if (len != 12 || memcmp(data, "SGU1", 4) || le32(data + 4) < 0x140 || le32(data + 4) > GLASSES_APP_LIMIT - GLASSES_APP_ADDRESS) error = 0x0D;
        else {
            expected_size = le32(data + 4); expected_crc = le32(data + 8);
            received = word_used = 0; word = 0xFFFFFFFFFFFFFFFFull;
            erase_page = GLASSES_APP_ADDRESS; erase_manifest = true;
            uploading = true; commit = false;
        }
    } else if (kind == 2) {
        if (!uploading || erase_manifest || len < 5 || len > 20 || le32(data) != received ||
            received + len - 4 > expected_size) error = 0x0D;
    } else if (kind == 3) {
        if (!uploading || len != 4 || memcmp(data, "END1", 4) || received != expected_size) error = 0x0D;
        else { commit = true; verify_at = 0; crc = 0xFFFFFFFFu; metadata_word = 0; }
    } else error = 0x0D;
    if (error) { aci_gatt_write_resp(conn, attr, 1, error, 0, (uint8_t *)data); return; }
    memcpy(pending, data, len); pending_len = len; pending_kind = kind;
    pending_conn = conn; pending_attr = attr; request_pending = true; packet_at = 4;
    last = HAL_GetTick();
}
void Glasses_OtaProcess(void) {
    int result;
    uint32_t now = HAL_GetTick();
    if (rebooting) { if ((int32_t)(now - reboot_at) >= 0) NVIC_SystemReset(); return; }
    if (uploading && (uint32_t)(now - last) >= 10000) { if (request_pending) reply(0x0E); Glasses_OtaDisconnected(); }
    if (!request_pending) {
        /* Disconnect/timeout callbacks only mark cancellation. The CPU2
         * cleanup command runs here, outside the BLE event handler. */
        (void)Glasses_FlashEndErase(); return;
    }
    if (pending_kind == 1) {
        uint32_t end = GLASSES_APP_ADDRESS + ((expected_size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
        if (erase_manifest) {
            result = Glasses_FlashErase(GLASSES_META_ADDRESS);
            if (result < 0) { reply(0x0E); return; }
            if (!result) erase_manifest = false;
            return;
        }
        if (erase_page < end) {
            result = Glasses_FlashErase(erase_page);
            if (result < 0) { reply(0x0E); return; }
            if (!result) erase_page += FLASH_PAGE_SIZE;
            return;
        }
        result = Glasses_FlashEndErase();
        if (result < 0) reply(0x0E);
        else if (!result) reply(0);
        return;
    }
    if (pending_kind == 2) {
        if (word_used == 8) {
            result = Glasses_FlashWrite(GLASSES_APP_ADDRESS + received - 8, word);
            if (result < 0) { reply(0x0E); return; }
            if (result) return;
            word_used = 0; word = 0xFFFFFFFFFFFFFFFFull;
        }
        while (packet_at < pending_len && word_used < 8) {
            ((uint8_t *)&word)[word_used++] = pending[packet_at++]; ++received;
        }
        if (packet_at == pending_len && word_used < 8) reply(0);
        return;
    }
    if (commit) {
        if (word_used) {
            result = Glasses_FlashWrite(GLASSES_APP_ADDRESS + received - word_used, word);
            if (result < 0) { reply(0x0E); return; }
            if (!result) word_used = 0;
            return;
        }
        if (verify_at < expected_size) {
            uint32_t n = expected_size - verify_at; if (n > 512) n = 512;
            crc = glasses_crc32(crc, (const uint8_t *)(GLASSES_APP_ADDRESS + verify_at), n);
            verify_at += n; return;
        }
        if ((crc ^ 0xFFFFFFFFu) != expected_crc ||
            !glasses_image_vectors_valid(*(const uint32_t *)GLASSES_APP_ADDRESS,
                    *(const uint32_t *)(GLASSES_APP_ADDRESS + 4), expected_size)) { reply(0x0E); return; }
        /* Program validity marker LAST. An interrupted download/verification remains invalid. */
        word = metadata_word == 0 ? ((uint64_t)expected_crc << 32) | expected_size :
                                   ((uint64_t)1 << 32) | GLASSES_IMAGE_MAGIC;
        result = Glasses_FlashWrite(GLASSES_META_ADDRESS + 8 * metadata_word, word);
        if (result < 0) { reply(0x0E); return; }
        if (result) return;
        if (++metadata_word == 2) {
            /* This warm reset must launch the verified image, even if the
             * separately powered touch controller still reports an old touch. */
            if (!boot_request_write(GLASSES_BOOT_APPLICATION)) { reply(0x0E); return; }
            glasses_fault[0] = 0; reply(0); uploading = commit = false;
            rebooting = true; reboot_at = now + 500;
        }
    }
}
/* Run before CPU2 is started. Never jump to an unverified or partial image. */
#if defined(__CC_ARM)
__asm static void jump_to_application(uint32_t sp, uint32_t entry) {
    MSR MSP, r0
    CPSIE i
    BX r1
}
#endif
void Glasses_BootTryApplication(void) {
    const GlassesImage *image = (const GlassesImage *)GLASSES_META_ADDRESS;
    uint32_t sp = *(const uint32_t *)GLASSES_APP_ADDRESS;
    uint32_t entry = *(const uint32_t *)(GLASSES_APP_ADDRESS + 4);
    uint8_t pads = 0, fresh_pads = 0;
    uint32_t request;
    HAL_PWR_EnableBkUpAccess();
    request = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6);
    if (request == GLASSES_BOOT_REQUEST || request == GLASSES_BOOT_APPLICATION) {
        if (!boot_request_write(0)) return;
        if (request == GLASSES_BOOT_REQUEST) return;
    }
    if ((glasses_fault[0] == 0x53474631 && glasses_fault[1]) || (glasses_reset_flags & RCC_CSR_IWDGRSTF)) return;
    if (request != GLASSES_BOOT_APPLICATION && CAP1203_Init(&hi2c1) == HAL_OK) {
        /* CAP1203 status can contain a released touch until INT is cleared.
         * Its datasheet explicitly requires two polls to observe a release.
         * Allow the first conversion (up to 200 ms), then confirm a held pad. */
        HAL_Delay(200);
        if (CAP1203_ReadTouch(&pads) == HAL_OK && (pads & 2)) {
            HAL_Delay(200);
            if (CAP1203_ReadTouch(&fresh_pads) == HAL_OK && (fresh_pads & 2)) return;
        }
    }
    if (image->magic != GLASSES_IMAGE_MAGIC || image->format != 1 ||
        !glasses_image_vectors_valid(sp, entry, image->size) ||
        (glasses_crc32(0xFFFFFFFFu, (const uint8_t *)GLASSES_APP_ADDRESS, image->size) ^ 0xFFFFFFFFu) != image->crc) return;
#ifdef GLASSES_HOST_TEST
    extern void Glasses_HostJumpApplication(uint32_t sp, uint32_t entry);
    Glasses_HostJumpApplication(sp, entry);
#else
    unsigned i;
    HAL_DeInit(); __disable_irq(); SysTick->CTRL = 0;
    for (i = 0; i < 8; ++i) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    SCB->VTOR = GLASSES_APP_ADDRESS; __DSB(); __ISB();
    /* No C code may execute after replacing MSP. */
#if defined(__CC_ARM)
    jump_to_application(sp, entry);
#else
    __asm volatile ("msr msp, %0\n cpsie i\n bx %1" : : "r"(sp), "r"(entry) : "memory");
    __builtin_unreachable();
#endif
#endif
}
#else
void Glasses_OtaPrepareConnection(uint16_t conn, uint16_t interval) { (void)conn; (void)interval; }
void Glasses_OtaInit(void) {}
void Glasses_OtaProcess(void) {}
void Glasses_OtaDisconnected(void) {}
void Glasses_BootTryApplication(void) {}
#endif
