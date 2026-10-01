#include "glasses_ble.h"
#include "main.h"
#include "app_conf.h"
#include "ble.h"

#define NO_CONNECTION 0xffffu
#define RETRY_MS 1000u
static bool initialized, advertising;
static uint16_t connection = NO_CONNECTION;
static uint32_t retry_at;
static const uint8_t name[] = { AD_TYPE_COMPLETE_LOCAL_NAME,
    'S', 'M', 'R', 'T', '_', 'G', 'L', 'A', 'S', 'S' };

void Glasses_BleInit(void)
{
    connection = NO_CONNECTION;
    advertising = false;
    retry_at = HAL_GetTick();
    initialized = true;
}

void Glasses_BleProcess(void)
{
    uint32_t now = HAL_GetTick();
    if (!initialized || connection != NO_CONNECTION || advertising ||
        (int32_t)(now - retry_at) < 0) return;
    retry_at = now + RETRY_MS;
    /* Clear a partially started advertisement before retrying. A controller
       already in standby may reject a stop; only a successful start below
       establishes our advertising state. Other stop errors defer the attempt. */
    uint8_t stopped = aci_gap_set_non_discoverable();
    if (stopped != BLE_STATUS_SUCCESS && stopped != BLE_STATUS_NOT_ALLOWED) return;
    /* Supply the name in the start command itself. No second update command
       can hide a failed start or leave a nameless advertisement behind. */
    if (aci_gap_set_discoverable(ADV_TYPE,
        CFG_FAST_CONN_ADV_INTERVAL_MIN, CFG_FAST_CONN_ADV_INTERVAL_MAX,
        CFG_BLE_ADDRESS_TYPE, ADV_FILTER, sizeof(name), name,
        0, 0, 0, 0) == BLE_STATUS_SUCCESS) advertising = true;
}

bool Glasses_BleConnected(uint8_t status, uint16_t handle)
{
    if (!initialized || connection != NO_CONNECTION) return false;
    if (status != BLE_STATUS_SUCCESS || handle == NO_CONNECTION) {
        advertising = false;
        retry_at = HAL_GetTick();
        return false;
    }
    connection = handle;
    advertising = false;
    return true;
}

bool Glasses_BleDisconnected(uint8_t status, uint16_t handle)
{
    if (!initialized || status != BLE_STATUS_SUCCESS ||
        connection == NO_CONNECTION || handle != connection) return false;
    connection = NO_CONNECTION;
    advertising = false;
    retry_at = HAL_GetTick();
    return true;
}

bool Glasses_BleAdvertising(void) { return advertising; }
