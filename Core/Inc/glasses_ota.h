#ifndef GLASSES_OTA_H
#define GLASSES_OTA_H
#include <stdbool.h>
#include <stdint.h>
#define GLASSES_APP_ADDRESS 0x08010000u
#define GLASSES_APP_LIMIT 0x08040000u
#define GLASSES_KEYS_ADDRESS 0x0800E000u
#define GLASSES_META_ADDRESS 0x0800F000u
#define GLASSES_IMAGE_MAGIC 0x53475531u
#define GLASSES_BOOT_REQUEST 0x4F544131u
#define GLASSES_BOOT_APPLICATION 0x41505031u
typedef struct { uint32_t size, crc, magic, format; } GlassesImage;
void Glasses_OtaInit(void);
void Glasses_OtaPrepareConnection(uint16_t conn, uint16_t interval);
void Glasses_OtaProcess(void);
void Glasses_OtaDisconnected(void);
void Glasses_OtaReboot(void);
void Glasses_BootTryApplication(void);
/* Configure CPU2 timing protection before BLE/key provisioning. Fail closed. */
bool Glasses_FlashInit(void);
/* 0 done, 1 busy: retry from foreground, -1 hardware error. */
int Glasses_FlashErase(uint32_t address);
/* Keep erase activity enabled across retries/pages. End it from foreground
 * before writes, or after an aborted erase. Returns 0/1/-1 like Erase. */
int Glasses_FlashEndErase(void);
int Glasses_FlashWrite(uint32_t address, uint64_t data);
#endif
