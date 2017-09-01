#ifndef WIRE_H
#define WIRE_H
#include "lpc111x.h"
#define IN 1
#define OUT 0
#define HIGH 1
#define LOW 0
#define NULL 0
#define SYS_FREQ 48000000//in Hz
#define PWM0 0 //reference to CT32B0; enable this when using pwm on P15 P16 P4
#define PWM1 1 //reference to CT32B1; enable this when using pwm on P10 P11 P13
#define PWM2 2  //reference to CT16B0; enable this when using pwm on P1 P2
#define FALLING 0
#define RISING 1
#define SLAVE 0
#define MASTER 1
#define PORT(P) ((P>=9&&P<=18)?0x1:0x0)

typedef enum {FALSE,TRUE} bool;
typedef unsigned char uint8;

void PINCONF(int P,void (*F)(unsigned int volatile*, int),int V) ;

int BIT_S(int P) ;

void RSET(unsigned int volatile* R,int V) ;
void RCLEAR(unsigned int volatile *R,int V) ;
void IOFUN_SET(int P,int V) ;

// int PORT(int P) {return (P>=9&&P<=18)?0x1:0x0;}

 void GPIO_DIR_SET(unsigned int volatile* R,int P,int V);
 void PINDIR(int P, int V) ;

 //int GPIO_READ(unsigned int volatile* R,int P) ;
 void GPIO_DATA_SET(unsigned int volatile* R,int P,int V) ;

 void digital_write(int P, int V) ;
 int digital_read(int P) ;
 void gpio_pin_mode(int P,int V);

 void pwm_enable(int C,int F) ;
 void pwm_write(int P,int D);
 void serial_begin(int baud);
 void serial_write_byte(uint8 data);
 void serial_write(const char* str);
 void serial_writenl(const char* str);
 void serial_irq(void (*func)());
 void serial_read_buff(uint8* buff,int nbyte);
void serial_write_hex(uint8 data);
void serial_write_int(int data);
void serial_write_buff(uint8* data, int nbyte, bool HEX);
 uint8 serial_read();
 int serial_available();
 void analog_init();
 void analog_enable(int p);
 int analog_read(int p);
 void delay_us(int value);
 void delay_ms(int value);
 void delay_s(int value);
 void sonar_init(int triger, int echo);
 int sonar_get_distance(int triger, int echo);
 void servo_init(int c,int p);
 void servo_write(int p,int angle);
 void spi_init(int mode);
 void spi_send(uint8* data, int length);
 void spi_receive(uint8* buff, int size);
 int __div_and_round(int a,int b);
#endif
