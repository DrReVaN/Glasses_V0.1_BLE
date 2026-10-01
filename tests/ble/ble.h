#include <stdint.h>
#define BLE_STATUS_SUCCESS 0
#define BLE_STATUS_NOT_ALLOWED 0x0c
#define AD_TYPE_COMPLETE_LOCAL_NAME 9
uint8_t aci_gap_set_non_discoverable(void);
uint8_t aci_gap_set_discoverable(uint8_t, uint16_t, uint16_t, uint8_t, uint8_t,
    uint8_t, const uint8_t *, uint8_t, const uint8_t *, uint16_t, uint16_t);
