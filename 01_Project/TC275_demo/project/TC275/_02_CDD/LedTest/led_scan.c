#include "led_scan.h"
#include "IfxPort_reg.h"

static volatile uint32 scan_counter;
static volatile uint32 scan_index;

static const uint8 scan_pins[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    0, 1, 2, 3, 4, 5, 6, 7
};

static const uint8 scan_is_p33[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1
};

#define SCAN_COUNT (sizeof(scan_pins)/sizeof(scan_pins[0]))

static void set_pin_output(Ifx_P *port, uint8 pin)
{
    uint32 iocr;
    if (pin < 8)
    {
        iocr = port->IOCR0.U;
        iocr &= ~(0xFu << (pin * 4));
        iocr |= (0x10u << (pin * 4));
        port->IOCR0.U = iocr;
    }
    else
    {
        iocr = port->IOCR4.U;
        iocr &= ~(0xFu << ((pin - 8) * 4));
        iocr |= (0x10u << ((pin - 8) * 4));
        port->IOCR4.U = iocr;
    }
}

static void set_pin_low(Ifx_P *port, uint8 pin)
{
    port->OMR.U = (1u << (pin + 16));
}

static void set_pin_high(Ifx_P *port, uint8 pin)
{
    port->OMR.U = (1u << pin);
}

static Ifx_P *get_port(uint8 is_p33)
{
    return is_p33 ? &MODULE_P33 : &MODULE_P00;
}

void LedScan_Init(void)
{
    uint32 i;
    for (i = 0; i < SCAN_COUNT; i++)
    {
        Ifx_P *p = get_port(scan_is_p33[i]);
        set_pin_output(p, scan_pins[i]);
        set_pin_low(p, scan_pins[i]);
    }
    scan_counter = 0;
    scan_index = 0;
}

void LedScan_Run1ms(void)
{
    scan_counter++;
    if (scan_counter >= 500)
    {
        Ifx_P *p;
        scan_counter = 0;

        p = get_port(scan_is_p33[scan_index]);
        set_pin_low(p, scan_pins[scan_index]);

        scan_index++;
        if (scan_index >= SCAN_COUNT)
            scan_index = 0;

        p = get_port(scan_is_p33[scan_index]);
        set_pin_high(p, scan_pins[scan_index]);
    }
}
