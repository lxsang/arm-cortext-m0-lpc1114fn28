#include "../core/rpicarapi.h"

int speed = 65;

void wall_hit(void)
{
	car_stop();
}


void serial_cmd(void)// this is the interup version
{
	if(serial_available())
	{
		char c = serial_read();
		switch(c)
		{
			case 'q':
			car_turn_left(speed,-1);
			break;
			
			case 'd':
			car_turn_right(speed,-1);
			break;
			
			case 'z':
			car_forward(speed,500);
			break;
			
			case 's':
			car_backward(speed,500);
			break;
			
			case 'x':
			car_stop();
			break;
			
			case 'w':
			car_forward_by_distance(speed,4);
			break;
			default:
			car_stop();
			break;
		}
		delay_ms(100);
	}
}

int main ()
{
	car_init();
	serial_begin(19200);
	serial_irq(serial_cmd);
	gpio_interrupt(28,RISING,wall_hit);
	while(1)
	{
		;;
	}
}