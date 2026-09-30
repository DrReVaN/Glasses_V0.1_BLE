#ifndef TEST_BLE_H
#define TEST_BLE_H
#include <stdint.h>
int aci_gatt_write_resp(uint16_t conn, uint16_t attr, uint8_t status, uint8_t error, uint8_t len, uint8_t *data);
int aci_l2cap_connection_parameter_update_req(uint16_t conn, uint16_t min, uint16_t max, uint16_t latency, uint16_t timeout);
#endif
