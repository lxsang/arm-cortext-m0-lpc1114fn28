#include "../core/rpicarapi.h"
#define IR_L_MARG 680
#define IR_R_MARG 890
int speed = 90;
int delay = 100;
typedef enum {STOP, FORDWARD, BACKWARD, LEFT,RIGHT} STATE;
STATE current_state = STOP;
int force_stop;
void wall_hit(void)
{
	car_stop();
}
void car_state()
{
	int data[3];
	int left_margin;
	int right_margin;
	car_sensors_read(data);
	left_margin = data[1] - IR_L_MARG;
	right_margin = data[2] - IR_R_MARG;
	//log
	serial_write("sonar= ");
	serial_write_int(data[0]);
	serial_write(", IRL= ");
	serial_write_int(data[1]);
	serial_write(", IRR= ");
	serial_write_int(data[2]);
	serial_writenl("");
	
	if (data[0] > 22 && left_margin>0 && right_margin>0)
	{
		current_state = FORDWARD;
		return;
	} 
	if(right_margin > 0 && right_margin > left_margin)
	{
		current_state = RIGHT;
		return;
	}
	if(left_margin>0 && left_margin>right_margin)
	{
		current_state = LEFT;
		return;
	}
	if (data[0] <= 22)
	{
		current_state = BACKWARD;
		return;
	}
	current_state = STOP;
}
void car_cmd()
{
	switch(current_state)
	{
		case STOP:
		car_stop();
		break;
		
		case FORDWARD:
		car_forward(speed,delay);
		break;
		
		case BACKWARD:
		car_backward(speed,delay);
		break;
		
		case LEFT:
		car_turn_left(speed,delay);
		break;
		
		case RIGHT:
		car_turn_right(speed,delay);
		break;
		
		default:
		car_stop();
		break;
	}
}

void serial_cmd(void)// this is the interup version
{
	if(serial_available())
	{
		char c = serial_read();
		switch(c)
		{
			case 's':
			force_stop = 1;
			break;
			
			case 'z':
			force_stop=0;
			break;
			
			default:
			force_stop = 0;
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
	force_stop = 0;
	gpio_interrupt(28,RISING,wall_hit);
	serial_write("Init every thing \n");
	int i;
	while(1)
	{
		if(!force_stop)
		{
			car_state();
			car_cmd();
		}
		else
		{
			car_stop();
		}
		/*serial_write(", IRL= ");
		serial_write_int(analog_read(IR_LEFT));
		serial_write(", IRR= ");
		serial_write_int(analog_read(IR_RIGHT));
		serial_writenl("");
		*/
		//delay_ms(100);
	}
}