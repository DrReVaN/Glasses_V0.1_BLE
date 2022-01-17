/*
Header File / Definitions
Library for Cap1203 Capacitiv Sensor
Developed for Keil MDK ARM
Developed by Lukas Hummer

Version 0.1

* Copyright (C) 2021 - L. Hummer
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#include "stm32wbxx_hal.h"

//Register

#define MAIN_CONTROL  0x00
#define GENERAL_STATUS  0x02
#define SENSOR_INPUT_STATUS  0x03
#define NOISE_FLAG_STATUS  0x0A
#define SENSOR_INPUT_1_DELTA_COUNT  0x10
#define SENSOR_INPUT_2_DELTA_COUNT  0X11
#define SENSOR_INPUT_3_DELTA_COUNT  0X12
#define SENSITIVITY_CONTROL  0x1F
#define CONFIG  0x20
#define SENSOR_INPUT_ENABLE  0x21
#define SENSOR_INPUT_CONFIG  0x22
#define SENSOR_INPUT_CONFIG_2  0x23
#define AVERAGING_AND_SAMPLE_CONFIG  0x24
#define CALIBRATION_ACTIVATE_AND_STATUS  0x26
#define INTERRUPT_ENABLE  0x27
#define REPEAT_RATE_ENABLE  0x28
#define MULTIPLE_TOUCH_CONFIG  0x2A
#define MULTIPLE_TOUCH_PATTERN_CONFIG  0x2B
#define MULTIPLE_TOUCH_PATTERN  0x2D
#define BASE_COUNT_OUT  0x2E
#define RECALIBRATION_CONFIG  0x2F
#define SENSOR_1_INPUT_THRESH  0x30
#define SENSOR_2_INPUT_THRESH  0x31
#define SENSOR_3_INPUT_THRESH  0x32
#define SENSOR_INPUT_NOISE_THRESH  0x38
#define STANDBY_CHANNEL  0x40
#define STANDBY_CONFIG  0x41
#define STANDBY_SENSITIVITY  0x42
#define STANDBY_THRESH  0x43
#define CONFIG_2 = 0x44
#define SENSOR_INPUT_1_BASE_COUNT  0x50
#define SENSOR_INPUT_2_BASE_COUNT  0x51
#define SENSOR_INPUT_3_BASE_COUNT  0x52
#define POWER_BUTTON  0x60
#define POWER_BUTTON_CONFIG  0x61
#define SENSOR_INPUT_1_CALIBRATION  0xB1
#define SENSOR_INPUT_2_CALIBRATION  0xB2
#define SENSOR_INPUT_3_CALIBRATION  0xB3
#define SENSOR_INPUT_CALIBRATION_LSB_1  0xB9
#define PROD_ID  0xFD
#define MANUFACTURE_ID  0xFE
#define REVISION  0xFF


// Declare I2C Address
#define CAP1203_I2C_ADDR 0x28

// Register values

// Product ID - always the same (pg. 22)
#define PROD_ID_VALUE 0x6D

// Capacitive sensor input (pg. 23)
#define OFF 0x00 // No touch detecetd
#define ON 0x01  // Check capacitive sensor input (pg. 23)

// Pads to be power buttion (pg. 43)
#define PWR_CS1 0x00 // Pad 1 (Left)
#define PWR_CS2 0x01 // Pad 2 (Middle)
#define PWR_CS3 0x02 // Pad 3 (Right)

//#define PAD_LEFT PWR_CS1
//#define PAD_MIDDLE PWR_CS2
//#define PAD_RIGHT PWR_CS3

// Power button hold time to generate interrupt (pg. 44)
#define PWR_TIME_280_MS 0x00  // 280 ms
#define PWR_TIME_560_MS 0x01  // 560 ms
#define PWR_TIME_1120_MS 0x02 // 1.12 sec
#define PWR_TIME_2240_MS 0x03 // 2.24 sec

// Sensitivity for touch detection (pg. 25)
#define SENSITIVITY_128X 0x00 // Most sensitive
#define SENSITIVITY_64X 0x01
#define SENSITIVITY_32X 0x02
#define SENSITIVITY_16X 0x03
#define SENSITIVITY_8X 0x04
#define SENSITIVITY_4X 0x05
#define SENSITIVITY_2X 0x06
#define SENSITIVITY_1X 0x07 // Least sensitive

// Cycle time 
#define CYCLETIME_35ms 0x38
#define CYCLETIME_70ms 0x39
#define CYCLETIME_105ms 0x3A
#define CYCLETIME_140ms 0x3B
// Interrupt Enable Channels
#define ALL_CS_ENB 0x07 //Alle Channel aktiv
#define CS1_ENB 0x01
#define CS2_ENB 0x02
#define CS3_ENB 0x04


static void write_register(uint8_t reg, uint8_t data);
void CAP1203_Init(I2C_HandleTypeDef i2c_handler);
static uint8_t read_register(uint8_t reg);

void CAP1203_Set_Sensitivity(uint8_t value);
uint8_t CAP1203_Get_CS_State(void);
uint8_t CAP1203_Get_CS1_State(void);
uint8_t CAP1203_Get_CS2_State(void);
uint8_t CAP1203_Get_CS3_State(void);


void CAP1203_Int_Enb(uint8_t channels);
void CAP1203_Int_Clr(void);

void CAP1203_Go_Standby(uint8_t cycletime);
void CAP1203_Exit_Standby(void);
void CAP1203_Set_Standby_CS(uint8_t channels);
void CAP1203_Set_Standby_Sensitivity(uint8_t value);
static void CAP1203_Set_Standby_Cycletime(uint8_t value);

void CAP1203_Go_PWR(void);
void CAP1203_Exit_PWR(void);
void CAP1203_Set_PWR_CS(uint8_t channel);
void CAP1203_Set_PWR_Time(uint8_t pwr_time);


uint8_t CAP1203_DeltaCount2(void);
uint8_t CAP1203_MainControl(void);










