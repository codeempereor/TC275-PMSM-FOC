#ifndef LED_TEST_H
#define LED_TEST_H

#include "Platform_Types.h"

void LedTest_Init(void);
void LedTest_SetLed1(uint8 on);
void LedTest_SetLed2(uint8 on);
void LedTest_ToggleLed1(void);
void LedTest_ToggleLed2(void);
void LedTest_Run1ms(void);
void KeyTest_Init(void);
uint8 KeyTest_Read(void);

#endif
