/*The LED is connected to the dp2 of the chip (PIO0_9)*/
#include "lpc111x.h"
  
int main()
{
    // Turn on clock for GPIO, IOCON
    SYSAHBCLKCTRL |= BIT6  + BIT16;
    // power on GPIO function on PIO0_9
    IOCON_PIO0_9 &= ~(0x7);
    // configure PIO0_9 as output
    GPIO0DIR |= (1<<9); 
    // turn of the PIO0_9
    GPIO0DATA &= ~(1<<9);
  
    int n;
    while(1)
    {
        // turn on the led
        GPIO0DATA |= (1<<9); 
        n=1000000; while(--n);
        //turn off the led
        GPIO0DATA &= ~(1<<9); 
        n=1000000; while(--n);
    } 
}