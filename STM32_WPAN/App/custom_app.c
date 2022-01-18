/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* DVC_INFO_SVC */
  /* RECEIVE_SVC */
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */
uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

uint8_t SecureReadData;

char helparray[128] = {0};
extern char time[] ;
extern char date[];
char helparraypush[128] = {0};

extern uint8_t newMessage;
extern char text[];
uint8_t PushIndex = 0;
uint8_t PushIndexLength = 0;

uint8_t rxCounter = 0;
uint8_t maxRx = 0;

extern uint8_t bleConnected;
extern int8_t page;
extern uint8_t EventFlag;
extern uint8_t powerOff;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
  /* DVC_INFO_SVC */
  /* RECEIVE_SVC */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch(pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

  /* DVC_INFO_SVC */
    case CUSTOM_STM_DVC_FW_NR_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DVC_FW_NR_READ_EVT */

      /* USER CODE END CUSTOM_STM_DVC_FW_NR_READ_EVT */
      break;

    case CUSTOM_STM_DVC_NAME_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DVC_NAME_READ_EVT */

      /* USER CODE END CUSTOM_STM_DVC_NAME_READ_EVT */
      break;

  /* RECEIVE_SVC */
    case CUSTOM_STM_TIME_UPDATE_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TIME_UPDATE_WRITE_NO_RESP_EVT */
			memset(helparray, 0, 128);		//set array to 0
			for(int i=0;i<pNotification->DataTransfered.Length;i++) {
				helparray[i] = pNotification->DataTransfered.pPayload[i];
				
			}
			time[0] = helparray[0];	
			time[1] = helparray[1];
			time[2] = helparray[2];
			time[3] = helparray[3];
			
			date[0] = helparray[4];
			date[1] = helparray[5];
			date[2] = helparray[6];
			date[3] = helparray[7];
			
      /* USER CODE END CUSTOM_STM_TIME_UPDATE_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_PUSH_NOTIFICATION_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PUSH_NOTIFICATION_WRITE_NO_RESP_EVT */
		
			if(pNotification->DataTransfered.pPayload[0] == 0x00) {
				memset(text, 0, 128);		//set array to 0
				maxRx = pNotification->DataTransfered.pPayload[1];
				rxCounter = 0;
				PushIndex = 0;
			}
			
			PushIndexLength = 0;
			
			if(pNotification->DataTransfered.pPayload[0] == rxCounter){
				if(powerOff == 0 && rxCounter == 0) {
					newMessage = 1;
					page = 1;
					EventFlag = 2;
				}
				rxCounter++;
				
				for(;PushIndexLength<19;PushIndexLength++) {
					/*if(pNotification->DataTransfered.pPayload[PushIndexLength+2] == 0x00 || pNotification->DataTransfered.pPayload[PushIndexLength+2] == 0xFF) {
						PushIndex--;
					}
					else {*/
					text[PushIndex] = pNotification->DataTransfered.pPayload[PushIndexLength+2];
					//}
					PushIndex++;
				}
			}
			

			//PushIndex++;

			
			//TODO: handle more than 20byte
			/*if(pNotification->DataTransfered.pPayload[1] > 1) {
				for(;PushIndex<18*pNotification->DataTransfered.pPayload[1];PushIndex++) {
				text[PushIndex] = pNotification->DataTransfered.pPayload[(PushIndex%20)+2];
				}
			}
			else {
				for(;PushIndex<pNotification->DataTransfered.Length-2;PushIndex++) {
				text[PushIndex] = pNotification->DataTransfered.pPayload[(PushIndex%20)+2];
				}
			}*/
			
			
			
      /* USER CODE END CUSTOM_STM_PUSH_NOTIFICATION_WRITE_NO_RESP_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch(pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
			bleConnected = 1;
			page = 4;
			if(powerOff != 1) {
				EventFlag = 2;
			}
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
			bleConnected = 0;
			page = 3;
			if(powerOff != 1) {
				EventFlag = 2;
			} 
      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

  /* DVC_INFO_SVC */
  /* RECEIVE_SVC */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
