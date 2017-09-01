#include "../core/wire.h"

int main()
{
    gpio_pin_mode(1,OUT);
	gpio_pin_mode(14,IN);
    digital_write(1,LOW);
    int n;
    while(1)
    {
		if(digital_read(14))
		{
			digital_write(1,HIGH);
		}
		else
		{
			 digital_write(1,LOW);
		}
         // Turn on PIO0_8
       // n=1000000; while(--n);
           // Turn on PIO0_8
       // n=1000000; while(--n);
    }    
}