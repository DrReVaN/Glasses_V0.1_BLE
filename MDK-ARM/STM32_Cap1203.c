/*

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

#include "STM32_Cap1203.h"
#include "stm32wbxx_hal.h"

extern int Error;

static I2C_HandleTypeDef i2cx;
static uint8_t iData[2];

static void write_register(uint8_t reg, uint8_t data){
	iData[0] = reg;
	iData[1] = data;
	HAL_I2C_Master_Transmit(&i2cx, CAP1203_I2C_ADDR << 1, iData, 2, 100);
}

static uint8_t read_register(uint8_t reg){
	uint8_t data = 0;
	iData[0] = reg;
	HAL_I2C_Master_Transmit(&i2cx, CAP1203_I2C_ADDR <<1, iData, 1, 100);
	HAL_I2C_Master_Receive(&i2cx, CAP1203_I2C_ADDR << 1, &data, 1, 100);
	return data;
}

//Init Sensor
void CAP1203_Init(I2C_HandleTypeDef i2c_handler){
	i2cx = i2c_handler; //Intern den 12c handler zuweisen
	
	CAP1203_Set_Sensitivity(SENSITIVITY_1X);
	CAP1203_Int_Enb(ALL_CS_ENB); // Alle Channels sollen einen Interrupt auslösen am Anfang
	CAP1203_Int_Clr();


}

//---------- Interrupt Functions ----------

// Interrupt im normalen Betrieb aktivieren
void CAP1203_Int_Enb(uint8_t channels){
	write_register(INTERRUPT_ENABLE, channels);
}
//Interrupt löschen ohne die anderen Werte zu verändern
void CAP1203_Int_Clr(void){
	uint8_t data = 0;
	data= read_register(MAIN_CONTROL) & 0xFE; // Den gelesenen Wert behalten, nur das LSB löschen. LSB = INT Bit
  write_register(MAIN_CONTROL, data);
	
}

//---------- Common Functions ----------

// Sensitivity einstellen
void CAP1203_Set_Sensitivity(uint8_t value){
	value = (value << 4) | 0xF;
	write_register(SENSITIVITY_CONTROL, value);
}
//Abfragen welches Pad HIGH ist
uint8_t CAP1203_Get_CS_State(void){
return read_register(SENSOR_INPUT_STATUS); 	
}
//Abfrage ob Pad1 (CS1) HIGH ist
uint8_t CAP1203_Get_CS1_State(void){
	uint8_t data = 0;
	data= CAP1203_Get_CS_State(); 	
	
	if((data & 0x01) == ON){
		return ON;
	}
	else{
		return OFF;
	}
}
//Abfrage ob Pad2 (CS2) HIGH ist
uint8_t CAP1203_Get_CS2_State(void){
	uint8_t data = 0;
	data= CAP1203_Get_CS_State(); 	
	
	if((data & 0x02) == ON << 1){
		return ON;
	}
	else{
		return OFF;
	}
}
//Abfrage ob Pad3 (CS3) HIGH ist
uint8_t CAP1203_Get_CS3_State(void){
	uint8_t data = 0;
	data= CAP1203_Get_CS_State(); 	
	
	if((data & 0x04) == ON << 2 ){
		return ON;
	}
	else{
		return OFF;
	}
}


//---------- Standby Functions ----------

// Standby Channel aktivieren
void CAP1203_Set_Standby_CS(uint8_t channels){
	write_register(STANDBY_CHANNEL, channels);
}
// Sensitivity einstellen im Standby
void CAP1203_Set_Standby_Sensitivity(uint8_t value){
	//value = (value << 4) | 0xF;
	write_register(STANDBY_SENSITIVITY, value);
}
// Sensor in den Standby schicken
void CAP1203_Go_Standby(uint8_t cycletime){
	CAP1203_Set_Standby_Cycletime(cycletime); 
	write_register(MAIN_CONTROL, 0x20);
}
// Sensor  Standby verlassen
void CAP1203_Exit_Standby(void){
	write_register(MAIN_CONTROL, 0x00);
}
// Clock Time einstellen im Standby
static void CAP1203_Set_Standby_Cycletime(uint8_t value){
	write_register(STANDBY_CONFIG, value);
}

//---------- Power Button Functions ----------

//Channel, welcher Power Button Funktion bekommt auswählen
void CAP1203_Set_PWR_CS(uint8_t channel){
	write_register(POWER_BUTTON, channel);
}
// Power Button Zeit einstellen 
void CAP1203_Set_PWR_Time(uint8_t pwr_time){
	write_register(POWER_BUTTON_CONFIG, (pwr_time << 4) );
}
void CAP1203_Go_PWR(void){
	uint8_t data = 0;
	data= read_register(POWER_BUTTON_CONFIG) | 0x40; // PWR in Standby activieren
  write_register(POWER_BUTTON_CONFIG, data);	
	
	CAP1203_Go_Standby(CYCLETIME_140ms);
}
void CAP1203_Exit_PWR(void){
	uint8_t data = 0;
	data= read_register(POWER_BUTTON_CONFIG) & 0x3F; // Den gelesenen Wert behalten, nur Standby activ Bit löschen Bit6 (Bit0:7)
  write_register(POWER_BUTTON_CONFIG, data);	
}

uint8_t CAP1203_DeltaCount2(void){
return read_register(GENERAL_STATUS); 	
}
	
uint8_t CAP1203_MainControl(void){
return read_register(MAIN_CONTROL); 	
}
	
	
	
	















