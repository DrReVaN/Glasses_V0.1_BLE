#ifndef GLASSES_BLE_H
#define GLASSES_BLE_H
#include <stdbool.h>
#include <stdint.h>

/* Single peripheral link. HCI events only schedule work; commands run in the main loop. */
void Glasses_BleInit(void);
void Glasses_BleProcess(void);
bool Glasses_BleConnected(uint8_t status, uint16_t handle);
bool Glasses_BleDisconnected(uint8_t status, uint16_t handle);
bool Glasses_BleAdvertising(void);
#endif
