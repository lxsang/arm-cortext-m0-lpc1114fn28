#include "../core/wire.h"

void test_interrupt(void)
{
	serial_writenl("OOPS! I hit the wall");
}
int main ()
{
	serial_begin(19200);
	gpio_interrupt(28,RISING,test_interrupt);
	//
	//
	while(1)
	{
		//serial_write_int(digital_read(28));
		serial_writenl("nop");
		delay_ms(200);
	}
}