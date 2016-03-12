
#ifndef CONIO_STDIO
#error This program requires the lightweight console I/O library
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cpu.h>

void __delay_cycles(int value)
 {
	 __asm__ volatile 
   	 (
		//"sub r0,#2					\n\t"
		"__delay_us_wait:			\n\t"
		"	sub r0,#1				\n\t"
		"	bne __delay_us_wait		\n\t"
    );
}
void delay_us(int value)
{
	__delay_cycles(value*12);
}
 void delay_ms(int value)
 {
	 delay_us(value*1000);
 }
 void delay_s(int value)
 {
	 delay_ms(value*1000);
 }

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
