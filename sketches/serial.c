
#ifndef CONIO_STDIO
#error This program requires the lightweight console I/O library
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cpu.h>
#include "delay.h"

int main(void)
{
    cpu_init(48000000);
    gpio_configure(PIO1_3, GPIO_MODE_OUTPUT);
     gpio_write(PIO1_3, 0x01);
    serial_stdio(CONSOLE_PORT);
    int dv;
     gpio_write(PIO1_3, 0x00);
    char * str = "this is a test string \n";
    for (;;)
    {
        printf(str);
        delay_ms(500);
    }
}
