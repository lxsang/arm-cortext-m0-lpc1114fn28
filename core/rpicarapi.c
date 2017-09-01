#include "rpicarapi.h"

motor motor_right = {
	.en = 11,
	.dir1 = 26,
	.dir2 = 3,
};
motor motor_left = {
	.en = 13,
	.dir1 = 10,
	.dir2 = 14,
};



void motor_init(motor m)
{
	pwm_enable(PWM1,100);
	gpio_pin_mode(m.en,OUT);
	gpio_pin_mode(m.dir1,OUT);
	gpio_pin_mode(m.dir2,OUT); 
}
void motor_forward(motor m, int speed)
{
	digital_write(m.dir1,HIGH);
	digital_write(m.dir2,LOW);
	pwm_write(m.en,speed);
}
void motor_stop(motor m)
{
	pwm_write(m.en,0);
}
void motor_backward(motor m, int speed)
{
	digital_write(m.dir1,LOW);
	digital_write(m.dir2,HIGH);
	pwm_write(m.en,speed);
}

void car_turn_left(int speed, int delay)
{
	motor_forward(motor_right,R_MOTOR_CAL(speed));
	motor_backward(motor_left,speed);
	if(delay > 0)
	{
		delay_ms(delay);
		car_stop();
	}
}
void car_turn_right(int speed, int delay)
{
	motor_forward(motor_left,speed);
	motor_backward(motor_right,R_MOTOR_CAL(speed));
	if(delay > 0)
	{
		delay_ms(delay);
		car_stop();
	}
}
void car_forward(int speed, int delay)
{
	motor_forward(motor_left,speed);
	motor_forward(motor_right,R_MOTOR_CAL(speed));
	if(delay > 0)
	{
		delay_ms(delay);
		car_stop();
	}
}
void car_backward(int speed, int delay)
{
	motor_backward(motor_left,speed);
	motor_backward(motor_right,R_MOTOR_CAL(speed));
	if(delay > 0)
	{
		delay_ms(delay);
		car_stop();
	}
}
__wait_until_distance_reached(int distance)
{
	int fb_new = -1;
	int fb_last = -1;
	int fb;
	int count = 0;
	while(count < distance)
	{	
		fb = analog_read(MOTOR_L_FB);
		fb_new = (fb <= MOTOR_FB_TRIG) ? 1 : 0; 
		count = (fb_last != -1 && fb_new != fb_last) ? count+1:count;
		fb_last = fb_new;
		delay_ms(25);
	}
}
int car_forward_by_distance(int speed, int distance)
{
	car_forward(speed,-1);
	__wait_until_distance_reached(distance);
	car_stop();
}
int car_backward_by_distance(int speed, int distance)
{
	car_backward(speed,-1);
	__wait_until_distance_reached(distance);
	car_stop();
}
void car_stop()
{
	motor_stop(motor_right);
	motor_stop(motor_left);
}

void car_init()
{
	//init motor
	motor_init(motor_left);
	motor_init(motor_right);
	//init sensor
	// IR Sensor
	analog_init();
	analog_enable(IR_LEFT);
	analog_enable(IR_RIGHT);
	analog_enable(MOTOR_L_FB);
	//sonar sensor
	sonar_init(SONAR_TRIGGER,SONAR_ECHO);
	car_forward_by_distance(65,1);
}

void car_sensors_read(int *buff)
{
	buff[0] = sonar_get_distance(SONAR_TRIGGER,SONAR_ECHO);
	buff[1] = analog_read(IR_LEFT);
	buff[2] = analog_read(IR_RIGHT);
}