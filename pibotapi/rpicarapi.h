#include <cpu.h>
#define IR_LEFT			12
#define IR_RIGHT 		4
#define SONAR_TRIGGER 	18
#define SONAR_ECHO 		17
#define MOTOR_L_FB		9
#define MOTOR_FB_TRIG	500
#define R_MOTOR_CAL(S) (0.8*S)

#define BUFF_SIZE 64
//right: en 11, d1 3, d2 26
//left: en 13, d1 9, d2 10
typedef struct{
	int en;
	int dir1;
	int dir2;
}motor;

void motor_init(motor m);
void motor_forward(motor m, int speed);
void motor_stop(motor m);
void motor_backward(motor m,int speed);

void car_turn_left(int speed, int delay);
void car_turn_right(int speed, int delay);
void car_forward(int speed, int delay);
void car_backward(int speed, int delay);
int car_forward_by_distance(int speed, int distance);
int car_backward_by_distance(int speed, int distance);
void car_stop();
void car_init(); 
