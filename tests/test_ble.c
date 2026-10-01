#include "glasses_ble.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t now;
static unsigned stops, starts;
static uint8_t stop_result, start_result;
uint32_t HAL_GetTick(void) { return now; }
uint8_t aci_gap_set_non_discoverable(void) { stops++; return stop_result; }
uint8_t aci_gap_set_discoverable(uint8_t type, uint16_t min, uint16_t max,
    uint8_t address, uint8_t filter, uint8_t len, const uint8_t *name,
    uint8_t uuid_len, const uint8_t *uuid, uint16_t slave_min, uint16_t slave_max)
{
    starts++;
    assert(type == 0 && min == 0x80 && max == 0xa0 && address == 0 && filter == 0);
    assert(len == 11 && name[0] == 9 && memcmp(name + 1, "SMRT_GLASS", 10) == 0);
    assert(!uuid_len && !uuid && !slave_min && !slave_max);
    return start_result;
}
int main(void)
{
    Glasses_BleProcess(); assert(!starts && !stops); /* CPU2 not ready */
    Glasses_BleInit(); start_result = 0x0c; Glasses_BleProcess();
    assert(starts == 1 && !Glasses_BleAdvertising());
    now = 999; Glasses_BleProcess(); assert(starts == 1);
    now = 1000; stop_result = 0x46; Glasses_BleProcess(); assert(starts == 1);
    now = 2000; stop_result = 0x0c; start_result = 0; Glasses_BleProcess();
    assert(starts == 2 && Glasses_BleAdvertising());
    stop_result = 0;
    now += 12u * 60u * 60u * 1000u; Glasses_BleProcess(); assert(starts == 2);
    assert(!Glasses_BleConnected(0x3e, 0)); /* Failed establishment must re-advertise */
    Glasses_BleProcess(); assert(starts == 3 && Glasses_BleAdvertising());
    assert(Glasses_BleConnected(0, 0)); /* Zero is a valid connection handle */
    assert(!Glasses_BleDisconnected(0, 1)); assert(!Glasses_BleDisconnected(1, 0));
    assert(!Glasses_BleConnected(0x3e, 1));
    now += 100000; Glasses_BleProcess(); assert(starts == 3);
    assert(Glasses_BleDisconnected(0, 0)); assert(!Glasses_BleDisconnected(0, 0));
    Glasses_BleProcess(); assert(starts == 4 && Glasses_BleAdvertising());
    for (unsigned i = 0; i < 1000; ++i) {
        assert(Glasses_BleConnected(0, 7)); assert(Glasses_BleDisconnected(0, 7));
        Glasses_BleProcess(); assert(Glasses_BleAdvertising());
    }
    now = UINT32_MAX - 500; Glasses_BleInit(); start_result = 0x0c;
    Glasses_BleProcess(); unsigned before = starts;
    now = 498; Glasses_BleProcess(); assert(starts == before);
    now = 499; start_result = 0; Glasses_BleProcess(); assert(starts == before + 1);
    puts("BLE advertising failures, failed/stale link events, overnight idle, 1000 reconnects and tick wrap: OK");
}
