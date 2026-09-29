#ifndef GLASSES_APP_H
#define GLASSES_APP_H
#include "glasses_core.h"
extern volatile uint8_t glasses_touch_irq;
extern GlassesRx glasses_rx;
extern GlassesClock glasses_clock;
void Glasses_Init(void);
void Glasses_Process(void);
void Glasses_Connected(bool connected);
bool Glasses_PairingRequest(uint16_t handle, uint32_t value);
void Glasses_PairingDone(void);
void Glasses_RequestOta(void);
void Glasses_Fatal(uint32_t code);
void Glasses_WatchdogInit(void);
void Glasses_SecurityKeys(uint8_t irk[16], uint8_t erk[16]);
#endif
