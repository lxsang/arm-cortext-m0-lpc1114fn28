#include "../core/wire.h"

int main ()
{
	//serial_begin(19200);
	gpio_pin_mode(1,OUT);
	//while(!serial_available());
	while(1)
	{
		//serial_write("Interval delay: 5s\r\n");
		digital_write(1,HIGH);
		delay_s(5);
		digital_write(1,LOW);
		delay_s(5);
	}
}