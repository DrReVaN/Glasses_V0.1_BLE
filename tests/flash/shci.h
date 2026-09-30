#ifndef TEST_FLASH_SHCI_H
#define TEST_FLASH_SHCI_H
typedef enum { SHCI_Success, SHCI_Error } SHCI_CmdStatus_t;
typedef enum { FLASH_ACTIVITY_CONTROL_PES, FLASH_ACTIVITY_CONTROL_SEM7 } SHCI_C2_SET_FLASH_ACTIVITY_CONTROL_Source_t;
typedef enum { ERASE_ACTIVITY_OFF, ERASE_ACTIVITY_ON } SHCI_EraseActivity_t;
SHCI_CmdStatus_t SHCI_C2_SetFlashActivityControl(SHCI_C2_SET_FLASH_ACTIVITY_CONTROL_Source_t source);
SHCI_CmdStatus_t SHCI_C2_FLASH_EraseActivity(SHCI_EraseActivity_t activity);
#endif
