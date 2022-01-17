/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomDvc_Info_SvcHdle;                    /**< DVC_INFO_SVC handle */
  uint16_t  CustomDvc_Fw_NrHdle;                  /**< DVC_FW_NR handle */
  uint16_t  CustomDvc_NameHdle;                  /**< DVC_NAME handle */
  uint16_t  CustomReceive_SvcHdle;                    /**< RECEIVE_SVC handle */
  uint16_t  CustomTime_UpdateHdle;                  /**< TIME_UPDATE handle */
  uint16_t  CustomPush_NotificationHdle;                  /**< PUSH_NOTIFICATION handle */
}CustomContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static const uint8_t SizeDvc_Fw_Nr=4;
static const uint8_t SizeDvc_Name=32;
static const uint8_t SizeTime_Update=32;
static const uint8_t SizePush_Notification=128;
/**
 * START of Section BLE_DRIVER_CONTEXT
 */
PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */
extern int Trigger;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
/*
 The following 128bits UUIDs have been generated from the random UUID
 generator:
 D973F2E0-B19E-11E2-9E96-0800200C9A66: Service 128bits UUID
 D973F2E1-B19E-11E2-9E96-0800200C9A66: Characteristic_1 128bits UUID
 D973F2E2-B19E-11E2-9E96-0800200C9A66: Characteristic_2 128bits UUID
 */
#define COPY_DVC_INFO_SVC_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x10,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_DVC_FW_NR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x11,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_DVC_NAME_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x12,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_RECEIVE_SVC_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x20,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_TIME_UPDATE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x21,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_PUSH_NOTIFICATION_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x22,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_write_permit_req_event_rp0   *write_perm_req;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */
	
  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
		

      switch(blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          if(attribute_modified->Attr_Handle == (CustomContext.CustomTime_UpdateHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
						
						//Handle Char changes -> send Notification to custom_app.c
						Notification.Custom_Evt_Opcode = CUSTOM_STM_TIME_UPDATE_WRITE_NO_RESP_EVT;
						Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;		//Payload Array data in hex format
						Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Custom_STM_App_Notification(&Notification);
						
            /* USER CODE END CUSTOM_STM_Service_2_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if(attribute_modified->Attr_Handle == (CustomContext.CustomTime_UpdateHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if(attribute_modified->Attr_Handle == (CustomContext.CustomPush_NotificationHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
						
						//Handle Char changes -> send Notification to custom_app.c
						Notification.Custom_Evt_Opcode = CUSTOM_STM_PUSH_NOTIFICATION_WRITE_NO_RESP_EVT;
						Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;		//Payload Array data in hex format
						Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Custom_STM_App_Notification(&Notification);
						
            /* USER CODE END CUSTOM_STM_Service_2_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if(attribute_modified->Attr_Handle == (CustomContext.CustomPush_NotificationHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          write_perm_req = (aci_gatt_write_permit_req_event_rp0*)blecore_evt->data;
          if(write_perm_req->Attribute_Handle == (CustomContext.CustomTime_UpdateHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* Allow or reject a write request from a client using aci_gatt_write_resp(...) function */
            /*USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE */
						
              /* received a correct value for char */
              aci_gatt_write_resp(write_perm_req->Connection_Handle,
                                  write_perm_req->Attribute_Handle,
                                  0x00,    /* write_status = 0 (no error))*/
                                  0x00,    /* err_code */
                                  write_perm_req->Data_Length,
                                  (uint8_t *)&write_perm_req->Data[0]);
              /**
               * Notify the application to 
               */
							Notification.Custom_Evt_Opcode = CUSTOM_STM_TIME_UPDATE_WRITE_NO_RESP_EVT;
							Notification.DataTransfered.pPayload = write_perm_req->Data; 
							Notification.DataTransfered.Length = write_perm_req->Data_Length; 
              Custom_STM_App_Notification(&Notification);
            
            /*USER CODE END CUSTOM_STM_Service_2_Char_1_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE*/
          } /*if(write_perm_req->Attribute_Handle == (CustomContext.CustomTime_UpdateHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

          else if(write_perm_req->Attribute_Handle == (CustomContext.CustomPush_NotificationHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* Allow or reject a write request from a client using aci_gatt_write_resp(...) function */
            /*USER CODE BEGIN CUSTOM_STM_Service_2_Char_2_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE */
						
						aci_gatt_write_resp(write_perm_req->Connection_Handle,
                                  write_perm_req->Attribute_Handle,
                                  0x00,    /* write_status = 0 (no error))*/
                                  0x00,    /* err_code */
                                  write_perm_req->Data_Length,
                                  (uint8_t *)&write_perm_req->Data[0]);
              /**
               * Notify the application to 
               */
							Notification.Custom_Evt_Opcode = CUSTOM_STM_PUSH_NOTIFICATION_WRITE_NO_RESP_EVT;
							Notification.DataTransfered.pPayload = write_perm_req->Data; 
							Notification.DataTransfered.Length = write_perm_req->Data_Length; 
              Custom_STM_App_Notification(&Notification);
            /*USER CODE END CUSTOM_STM_Service_2_Char_2_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE*/
          } /*if(write_perm_req->Attribute_Handle == (CustomContext.CustomPush_NotificationHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;
        /* USER CODE BEGIN BLECORE_EVT */

        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */
					//trigger = 123;
          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  /*
   *          DVC_INFO_SVC
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for DVC_INFO_SVC +
   *                                2 for DVC_FW_NR +
   *                                2 for DVC_NAME +
   *                              = 5
   */

  COPY_DVC_INFO_SVC_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128,
                       (Service_UUID_t *) &uuid,
                       PRIMARY_SERVICE,
                       5,
                       &(CustomContext.CustomDvc_Info_SvcHdle));

  /**
   *  DVC_FW_NR
   */
  COPY_DVC_FW_NR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomDvc_Info_SvcHdle,
                    UUID_TYPE_128, &uuid,
                    SizeDvc_Fw_Nr,
                    CHAR_PROP_READ,
                    ATTR_PERMISSION_NONE,
                    GATT_DONT_NOTIFY_EVENTS,
                    0x10,
                    CHAR_VALUE_LEN_CONSTANT,
                    &(CustomContext.CustomDvc_Fw_NrHdle));
  /**
   *  DVC_NAME
   */
  COPY_DVC_NAME_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomDvc_Info_SvcHdle,
                    UUID_TYPE_128, &uuid,
                    SizeDvc_Name,
                    CHAR_PROP_READ,
                    ATTR_PERMISSION_NONE,
                    GATT_DONT_NOTIFY_EVENTS,
                    0x10,
                    CHAR_VALUE_LEN_VARIABLE,
                    &(CustomContext.CustomDvc_NameHdle));

  /*
   *          RECEIVE_SVC
   *
   * Max_Attribute_Records = 1 + 2*2 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for RECEIVE_SVC +
   *                                2 for TIME_UPDATE +
   *                                2 for PUSH_NOTIFICATION +
   *                              = 5
   */

  COPY_RECEIVE_SVC_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128,
                       (Service_UUID_t *) &uuid,
                       PRIMARY_SERVICE,
                       5,
                       &(CustomContext.CustomReceive_SvcHdle));

  /**
   *  TIME_UPDATE
   */
  COPY_TIME_UPDATE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomReceive_SvcHdle,
                    UUID_TYPE_128, &uuid,
                    SizeTime_Update,
                    CHAR_PROP_WRITE_WITHOUT_RESP,
                    ATTR_PERMISSION_NONE,
                    GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                    0x10,
                    CHAR_VALUE_LEN_VARIABLE,
                    &(CustomContext.CustomTime_UpdateHdle));
  /**
   *  PUSH_NOTIFICATION
   */
  COPY_PUSH_NOTIFICATION_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomReceive_SvcHdle,
                    UUID_TYPE_128, &uuid,
                    SizePush_Notification,
                    CHAR_PROP_WRITE_WITHOUT_RESP,
                    ATTR_PERMISSION_NONE,
                    GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                    0x10,
                    CHAR_VALUE_LEN_VARIABLE,
                    &(CustomContext.CustomPush_NotificationHdle));

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus result = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch(CharOpcode)
  {

    case CUSTOM_STM_DVC_FW_NR:
      result = aci_gatt_update_char_value(CustomContext.CustomDvc_Info_SvcHdle,
                                          CustomContext.CustomDvc_Fw_NrHdle,
                                          0, /* charValOffset */
                                          SizeDvc_Fw_Nr, /* charValueLen */
                                          (uint8_t *)  pPayload);
      /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_Service_1_Char_1*/
      break;

    case CUSTOM_STM_DVC_NAME:
      result = aci_gatt_update_char_value(CustomContext.CustomDvc_Info_SvcHdle,
                                          CustomContext.CustomDvc_NameHdle,
                                          0, /* charValOffset */
                                          SizeDvc_Name, /* charValueLen */
                                          (uint8_t *)  pPayload);
      /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_Service_1_Char_2*/
      break;

    case CUSTOM_STM_TIME_UPDATE:
      result = aci_gatt_update_char_value(CustomContext.CustomReceive_SvcHdle,
                                          CustomContext.CustomTime_UpdateHdle,
                                          0, /* charValOffset */
                                          SizeTime_Update, /* charValueLen */
                                          (uint8_t *)  pPayload);
      /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1*/

      /* USER CODE END CUSTOM_STM_Service_2_Char_1*/
      break;

    case CUSTOM_STM_PUSH_NOTIFICATION:
      result = aci_gatt_update_char_value(CustomContext.CustomReceive_SvcHdle,
                                          CustomContext.CustomPush_NotificationHdle,
                                          0, /* charValOffset */
                                          SizePush_Notification, /* charValueLen */
                                          (uint8_t *)  pPayload);
      /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_2*/

      /* USER CODE END CUSTOM_STM_Service_2_Char_2*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return result;
}
