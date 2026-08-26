#include "led_test.h"
#include "IfxPort_reg.h"

#define LED1_PIN 5
#define LED2_PIN 6

volatile uint32 led1_counter;
volatile uint32 led2_counter;

void LedTest_Init(void)
{
    uint32 iocr;

    iocr = P00_IOCR4.U;
    iocr &= ~(0xFu << 12);
    iocr |= (0x8u << 12);
    iocr &= ~(0xFu << 20);
    iocr |= (0x8u << 20);
    P00_IOCR4.U = iocr;

    P00_OUT.U |= (1u << LED1_PIN) | (1u << LED2_PIN);

    led1_counter = 0;
    led2_counter = 0;
}

void LedTest_SetLed1(uint8 on)
{
    if (on)
        P00_OUT.U &= ~(1u << LED1_PIN);
    else
        P00_OUT.U |= (1u << LED1_PIN);
}

void LedTest_SetLed2(uint8 on)
{
    if (on)
        P00_OUT.U &= ~(1u << LED2_PIN);
    else
        P00_OUT.U |= (1u << LED2_PIN);
}

void LedTest_ToggleLed1(void)
{
    P00_OUT.U ^= (1u << LED1_PIN);
}

void LedTest_ToggleLed2(void)
{
    P00_OUT.U ^= (1u << LED2_PIN);
}

void LedTest_Run1ms(void)
{
    led1_counter++;
    led2_counter++;

    if (led1_counter >= 500)
    {
        led1_counter = 0;
        LedTest_ToggleLed1();
    }

    if (led2_counter >= 250)
    {
        led2_counter = 0;
        LedTest_ToggleLed2();
    }
}

void KeyTest_Init(void)
{
    uint32 iocr = P00_IOCR4.U;
    iocr &= ~(0xFu << 28);
    iocr |= (0x2u << 28);
    P00_IOCR4.U = iocr;
}

uint8 KeyTest_Read(void)
{
    return (P00_IN.U & (1u << 7)) ? 0 : 1;
}
