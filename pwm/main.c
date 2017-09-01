#include "../core/wire.h"


int main ()
{
	int pin = 11;
	pwm_enable(PWM1,30000);
	pwm_write(pin,0);
	int duty = 50;
	int n;
	while(1)
	{
		pwm_write(pin,duty++);
		if(duty >100)
			duty = 0;
		n=10000; while(--n);
	}
}