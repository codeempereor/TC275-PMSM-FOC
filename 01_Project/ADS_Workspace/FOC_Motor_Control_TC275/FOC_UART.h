#ifndef FOC_UART_H
#define FOC_UART_H

#include "Ifx_Types.h"

void FOC_UART_Init(void);
void FOC_UART_SendChar(char c);
void FOC_UART_Print(const char *str);
void FOC_UART_PrintInt(int32 val);
void FOC_UART_PrintFloat(float32 val, int decimals);

#endif /* FOC_UART_H */
