/* Original ST-generated service scaffolding: Copyright (c) 2022 STMicroelectronics.
 * The original repository provides it AS-IS when no component LICENSE is included. */
/* Custom Smartglasses GATT service, UUIDs retained for the companion app.
 * All writes require encrypted, authenticated Secure Connections. ATT requests
 * are processed exactly once, with explicit errors and flow control. */
#include "common_blesvc.h"
#include "custom_stm.h"
#include "glasses_app.h"
#include "glasses_ota.h"
#include "glasses_version.h"
#include <string.h>
#if CFG_BONDING_MODE != 1 || CFG_SC_SUPPORT != CFG_SECURE_MANDATORY || CFG_ENCRYPTION_KEY_SIZE_MIN != 16
#error "Smartglasses requires bonded Secure Connections with 16-byte encryption keys"
#endif
static uint16_t info_svc, receive_svc, fw_char, name_char, time_char, push_char, boot_char, diag_char;
static uint16_t ota_svc, begin_char, data_char, end_char;
#ifdef SMARTGLASSES_BOOTLOADER
#define GLASSES_IMAGE_MODE 1
#else
#define GLASSES_IMAGE_MODE 0
#endif
/* Fixed image offset permits a manifest to be checked against the binary.
 * GATT identity appends the installed application's metadata size and CRC. */
__attribute__((used, section(".firmware_version"))) const uint8_t glasses_image_version[12] = {
    'S','G','V','1', GLASSES_VERSION_MAJOR & 255, GLASSES_VERSION_MAJOR >> 8,
    GLASSES_VERSION_MINOR & 255, GLASSES_VERSION_MINOR >> 8,
    GLASSES_VERSION_PATCH & 255, GLASSES_VERSION_PATCH >> 8,
    GLASSES_PROTOCOL_FORMAT, GLASSES_IMAGE_MODE
};
#ifdef SMARTGLASSES_BOOTLOADER
void Glasses_OtaWrite(uint8_t kind, uint16_t conn, uint16_t attr, const uint8_t *data, uint8_t len);
#endif
extern uint32_t glasses_fault[3], glasses_reset_flags;
static void uuid(uint8_t *out, uint32_t id, bool service) {
    static const uint8_t characteristic[] = {0x19,0xED,0x82,0xAE,0xED,0x21,0x4C,0x9D,0x41,0x45,0x22,0x8E};
    static const uint8_t svc[] = {0x8F,0xE5,0xB3,0xD5,0x2E,0x7F,0x4A,0x98,0x2A,0x48,0x7A,0xCC};
    memcpy(out, service ? svc : characteristic, 12);
    out[12] = (uint8_t)id; out[13] = (uint8_t)(id >> 8);
    out[14] = (uint8_t)(id >> 16); out[15] = (uint8_t)(id >> 24);
}
static void check(tBleStatus status) { if (status != BLE_STATUS_SUCCESS) Glasses_Fatal(10); }
static void add(uint16_t svc, uint32_t id, uint8_t len, bool write, bool protected_read, uint16_t *handle) {
    Char_UUID_t u;
    uuid(u.Char_UUID_128, id, false);
    check(aci_gatt_add_char(svc, UUID_TYPE_128, &u, len,
        write ? CHAR_PROP_WRITE : CHAR_PROP_READ,
        write ? ATTR_PERMISSION_AUTHEN_WRITE | ATTR_PERMISSION_ENCRY_WRITE :
        protected_read ? ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_ENCRY_READ : ATTR_PERMISSION_NONE,
        write ? GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP :
        protected_read ? GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP : GATT_DONT_NOTIFY_EVENTS,
        16, CHAR_VALUE_LEN_VARIABLE, handle));
}
static SVCCTL_EvtAckStatus_t handler(void *packet) {
    hci_event_pckt *evt = (hci_event_pckt *)((hci_uart_pckt *)packet)->data;
    evt_blecore_aci *vendor;
    if (evt->evt != HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE) return SVCCTL_EvtNotAck;
    vendor = (evt_blecore_aci *)evt->data;
    if (vendor->ecode == ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE) {
        aci_gatt_read_permit_req_event_rp0 *read = (void *)vendor->data;
        if (read->Attribute_Handle == diag_char + 1) {
            uint32_t diagnostic[5] = {glasses_reset_flags, glasses_fault[0] == 0x53474631 ? glasses_fault[1] : 0,
                glasses_rx.rejected, glasses_rx.dropped, glasses_rx.truncated};
            aci_gatt_update_char_value(info_svc, diag_char, 0, sizeof(diagnostic), (uint8_t *)diagnostic);
            aci_gatt_allow_read(read->Connection_Handle); return SVCCTL_EvtAckFlowEnable;
        }
    } else if (vendor->ecode == ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE) {
        aci_gatt_write_permit_req_event_rp0 *w = (void *)vendor->data;
        uint8_t error = 0;
#ifdef SMARTGLASSES_BOOTLOADER
        if (w->Attribute_Handle == begin_char + 1 || w->Attribute_Handle == data_char + 1 || w->Attribute_Handle == end_char + 1) {
            Glasses_OtaWrite(w->Attribute_Handle == begin_char + 1 ? 1 : w->Attribute_Handle == data_char + 1 ? 2 : 3,
                            w->Connection_Handle, w->Attribute_Handle, w->Data, w->Data_Length);
            return SVCCTL_EvtAckFlowEnable;
        }
#endif
        if (w->Attribute_Handle == boot_char + 1) {
            if (w->Data_Length != 4 || memcmp(w->Data, "OTA1", 4)) error = 0x0D;
            else Glasses_RequestOta();
        } else if (w->Attribute_Handle == time_char + 1) {
#ifdef SMARTGLASSES_BOOTLOADER
            error = 0x03;
#else
            if (!glasses_clock_set(&glasses_clock, w->Data, w->Data_Length, HAL_GetTick())) error = 0x0D;
#endif
        } else if (w->Attribute_Handle == push_char + 1) {
#ifdef SMARTGLASSES_BOOTLOADER
            error = 0x03;
#else
            if (!glasses_rx_push(&glasses_rx, w->Data, w->Data_Length, HAL_GetTick())) error = 0x0D;
#endif
        } else if (w->Attribute_Handle == begin_char + 1 || w->Attribute_Handle == data_char + 1 || w->Attribute_Handle == end_char + 1) error = 0x03;
        else return SVCCTL_EvtNotAck;
        aci_gatt_write_resp(w->Connection_Handle, w->Attribute_Handle, error != 0, error,
                            error ? 0 : w->Data_Length, w->Data);
        return SVCCTL_EvtAckFlowEnable;
    }
    return SVCCTL_EvtNotAck;
}
void SVCCTL_InitCustomSvc(void) {
    Service_UUID_t s;
    uint8_t version[20] = {0,2,0,0};
    uint8_t name[] = "Smartglasses";
#ifdef SMARTGLASSES_BOOTLOADER
    version[3] = 1;
#endif
    SVCCTL_RegisterSvcHandler(handler);
    uuid(s.Service_UUID_128, 0x10, true);
    check(aci_gatt_add_service(UUID_TYPE_128, &s, PRIMARY_SERVICE, 7, &info_svc));
    /* Extend the existing read value without shifting any GATT handles. */
    add(info_svc, 0x11, 20, false, false, &fw_char);
    add(info_svc, 0x12, 32, false, false, &name_char);
    add(info_svc, 0x13, 20, false, true, &diag_char);
    {
        const GlassesImage *installed = (const GlassesImage *)GLASSES_META_ADDRESS;
        memcpy(version + 4, glasses_image_version + 4, 7);
        uint32_t size = installed->magic == GLASSES_IMAGE_MAGIC && installed->format == 1 ? installed->size : 0;
        uint32_t crc = size ? installed->crc : 0;
        memcpy(version + 12, &size, 4); memcpy(version + 16, &crc, 4);
    }
    check(aci_gatt_update_char_value(info_svc, fw_char, 0, sizeof(version), version));
    check(aci_gatt_update_char_value(info_svc, name_char, 0, sizeof(name) - 1, name));
    uuid(s.Service_UUID_128, 0x20, true);
    check(aci_gatt_add_service(UUID_TYPE_128, &s, PRIMARY_SERVICE, 7, &receive_svc));
    add(receive_svc, 0x21, 12, true, false, &time_char);
    add(receive_svc, 0x22, 20, true, false, &push_char);
    add(receive_svc, 0x23, 4, true, false, &boot_char);
    /* OTA uses its own service, distinct from ST's unmodified BLE_Ota protocol. */
    uuid(s.Service_UUID_128, 0xFE20, true);
    check(aci_gatt_add_service(UUID_TYPE_128, &s, PRIMARY_SERVICE, 7, &ota_svc));
    add(ota_svc, 0xFE21, 12, true, false, &begin_char);
    add(ota_svc, 0xFE22, 20, true, false, &data_char);
    add(ota_svc, 0xFE23, 4, true, false, &end_char);
}
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t kind, uint8_t *p) {
    if (kind == CUSTOM_STM_DVC_FW_NR) return aci_gatt_update_char_value(info_svc, fw_char, 0, 4, p);
    if (kind == CUSTOM_STM_DVC_NAME) return aci_gatt_update_char_value(info_svc, name_char, 0, 32, p);
    return BLE_STATUS_INVALID_PARAMS;
}
