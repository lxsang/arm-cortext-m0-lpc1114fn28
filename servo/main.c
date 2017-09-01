#include "../core/wire.h"


int main ()
{
	int pin = 13;
	servo_init(PWM1,pin);
	serial_begin(19200);
	analog_init();
	analog_enable(10);
	while(1)
	{
		int vr = analog_read(10);
		serial_write_int(vr);
		serial_write("\r\n");
		//from 0-1023, map to 180 degree, so 6 value form a degree
		servo_write(pin,vr/6);//45 degree
		delay_ms(100);
	}
}