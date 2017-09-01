#include "../core/wire.h"

int count = 0; 

void serial_cmd()
{
	if(serial_available())
	{
		char c = serial_read();
		if(c = 'z')
			count = 0; 
	}
}

int main ()
{
	serial_begin(19200);
	serial_irq(serial_cmd);
	analog_init();
	//analog_enable(12);
	//analog_enable(4);
	analog_enable(9);
	int fb_new = -1;
	int fb_last = -1;
	int dist = 0;
	while(1)
	{
		fb_last = fb_new;
		fb_new = analog_read(9);
		serial_write_int(fb_last);
		serial_write(" ,  ");
		serial_write_int(fb_new);
		dist = abs(fb_new - fb_last);
		count = ( dist > 4) ? count+1:count;
		serial_write("   #   ");
		serial_write_int(count);
		serial_write("\r\n");
		
		delay_ms(100);
	}
}