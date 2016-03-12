
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
  int i;
  char buf[256];
  int x, y;

  cpu_init(48000000);

serial_stdio(CONSOLE_PORT);

  for (;;)
  {
    printf("This is a test string\n");
    delay_ms(500);
  }
}
