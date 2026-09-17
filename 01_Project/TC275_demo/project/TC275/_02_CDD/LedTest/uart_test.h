#ifndef UART_TEST_H
#define UART_TEST_H

#include "Platform_Types.h"

void UartTest_Init(void);
void UartTest_SendByte(uint8 data);
void UartTest_SendString(const char* str);
void UartTest_SendUint32(uint32 val);
void UartTest_SendHex(const char* label, uint32 val);

#endif
