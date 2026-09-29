/* Original ST-generated service scaffolding: Copyright (c) 2022 STMicroelectronics.
 * The original repository provides it AS-IS when no component LICENSE is included. */
/* Smartglasses application callbacks. The bounded parser lives in glasses_core.c. */
#include "app_common.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "glasses_app.h"
#include "glasses_ota.h"
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *event) {
    if (!event) return;
#ifndef SMARTGLASSES_BOOTLOADER
    if (event->Custom_Evt_Opcode == CUSTOM_STM_TIME_UPDATE_WRITE_NO_RESP_EVT)
        glasses_clock_set(&glasses_clock, event->DataTransfered.pPayload, event->DataTransfered.Length, HAL_GetTick());
    else if (event->Custom_Evt_Opcode == CUSTOM_STM_PUSH_NOTIFICATION_WRITE_NO_RESP_EVT)
        glasses_rx_push(&glasses_rx, event->DataTransfered.pPayload, event->DataTransfered.Length, HAL_GetTick());
#endif
}
void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *event) {
    if (event->Custom_Evt_Opcode == CUSTOM_CONN_HANDLE_EVT) Glasses_Connected(true);
    else if (event->Custom_Evt_Opcode == CUSTOM_DISCON_HANDLE_EVT) Glasses_Connected(false);
}
void Custom_APP_Init(void) { Glasses_OtaInit(); }
