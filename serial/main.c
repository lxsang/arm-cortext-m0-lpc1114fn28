#include "../core/wire.h"
void test(void)// this is the interup version
{
		if(serial_available())
		{
			uint8 v[2];
			serial_read_buff(v,2);
			serial_write("I receive: ");
			serial_write_buff(v,2,TRUE); 
			serial_write("\r\n");
		}
}

int main ()
{
	serial_begin(19200);
	serial_irq(test);
	gpio_pin_mode(1,OUT);
	while(1)
	{
		// if(serial_available())
// 		{
// 			uint8 v = serial_read();
// 			serial_write("I receive: ");
// 			serial_write_byte(v);
// 			serial_write("\r\n");
// 		}
		delay_ms(100);
		serial_writenl("nop");
		digital_write(1,HIGH);
	}
}