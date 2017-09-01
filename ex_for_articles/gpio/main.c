/*The LED is connected to the dp1 of the chip (PIO0_8)*/
#include "lpc111x.h"

void gpio1_irq(void)
{
	// if the interrupt is triggered on pin 5 of port 1
	if((GPIO1RIS & (1<<5)) > 0)
	{
		// toggle the led 
		GPIO0DATA ^= (1<<8);
		// clear the edge detection logic
		GPIO1IC = (1<<5);
	}
}
  
int main()
{
    // Turn on clock for GPIO, IOCON
    SYSAHBCLKCTRL |= BIT6  + BIT16;
   
   	// power on GPIO function on PIO0_8
   	IOCON_PIO0_8 &= ~(0x7);
	// configure PIO0_8 as output
	GPIO0DIR |= (1<<8);
  	// turn off the led
	GPIO0DATA &= ~(1<<8);
   	
	// power on GPIO function on PIO1_5
    IOCON_PIO1_5 &= ~(0x7); 
	// configure PIO1_5 as input
	GPIO1DIR &= ~(1<<5);
	// turn on interrupt function
	ISER |= (1<<30);
	// interrupt as edge sensitive
	GPIO1IS &= ~(1<<5);
  	// interrupt triggered on rising edge
	GPIO1IBE &= ~(1<<5);
	GPIO1IEV |= (1<<5);
	//enable interrupt on GPIO1_5
	GPIO1IC = (1<<5);
	GPIO1IE |= (1<<5);
	
    while(1)
    {
		// turn off the led
		//GPIO0DATA &= ~(1<<8);
		;;//do other stuff here
    } 
}
