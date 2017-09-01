#include "lpc111x.h"
#define IN 1
#define OUT 0
#define HIGH 1
#define LOW 0
#define DIGITAL 1
#define ANALOG 0
#define SYS_FREQ 48000000//in Hz
#define PWM0 0 //reference to CT32B0; enable this when using pwm on P15 P16 P4
#define PWM1 1 //reference to CT32B1; enable this when using pwm on P10 P11 P13
#define PWM2 2  //reference to CT16B0; enable this when using pwm on P1 P2
typedef enum {FALSE,TRUE} bool;
typedef unsigned char uint8;

#define PINCONF(P,F,V) (P==1?F(IOCON_PIO0_8,V):(P==2?F(IOCON_PIO0_9,V):(P==3?F(IOCON_SWCLK_PIO0_10,V):(P==4?F(IOCON_R_PIO0_11,V):( \
	P==5?F(IOCON_PIO0_5,V):(P==6?F(IOCON_PIO0_6,V):(P==9?F(IOCON_R_PIO1_0,V):(P==10?F(IOCON_R_PIO1_1,V):(P==11?F(IOCON_R_PIO1_2,V):( \
	P==12?F(IOCON_SWDIO_PIO1_3,V):(P==13?F(IOCON_PIO1_4,V):(P==14?F(IOCON_PIO1_5,V):(P==15?F(IOCON_PIO1_6,V):(P==16?F(IOCON_PIO1_7,V):( \
	P==17?F(IOCON_PIO1_8,V):(P==18?F(IOCON_PIO1_9,V):(P==23?F(IOCON_RESET_PIO0_0,V):(P==24?F(IOCON_PIO0_1,V):(P==25?F(IOCON_PIO0_2,V):(\
	P==26?F(IOCON_PIO0_3,V):(P==27?F(IOCON_PIO0_4,V):(P==28?F(IOCON_PIO0_7,V):0xFFF))))))))))))))))))))))

#define BIT_S(P) (P==1?8:(P==2?9:(P==3?10:(P==4?11:(P==5?5:(P==6?6:(P==9?0:(P==10?1:(P==11?2:( \
	P==12?3:(P==13?4:(P==14?5:(P==15?6:(P==16?7:( P==17?8:(P==18?9:(P==23?0:(P==24?1:(P==25?2:(\
	P==26?3:(P==27?4:(P==28?7:12)))))))))))))))))))))) 

#define RSET(R, V) (R |= V)
#define RCLEAR(R, V) (R &= (~V))
#define IOFUN_SET(P,V) do{PINCONF(P,RCLEAR,0x7);PINCONF(P,RSET,V);}while(0);
#define PORT(P) ((P>=9&&P<=18)?0x1:0x0)

#define GPIO_DIR_SET(R,P,V) (R = (V == IN)?(R&(~(1<<BIT_S(P)))):(R|(1<<BIT_S(P))))
#define PINDIR(P, V) ((PORT(P) == 0x1)?GPIO_DIR_SET(GPIO1DIR,P,V):GPIO_DIR_SET(GPIO0DIR,P,V))

#define GPIO_READ(R,P) (R & (1<<BIT_S(P))?HIGH:LOW)
#define GPIO_DATA_SET(R,P,V) (R = (V == HIGH)?(R|(1<<BIT_S(P))):(R&(~(1<<BIT_S(P)))))

#define digital_write(P, V) ((PORT(P) == 0x1)?GPIO_DATA_SET(GPIO1DATA,P,V):GPIO_DATA_SET(GPIO0DATA,P,V))
#define digital_read(P) ((PORT(P) == 0x1)?GPIO_READ(GPIO1DATA,P):GPIO_READ0(GPIO0DATA,P))
#define gpio_pin_mode(P,V) do{SYSAHBCLKCTRL |= BIT6 + BIT13 + BIT16;PINCONF(P,RCLEAR,0x7);\
		PINCONF(P,RSET,(P==3||P==4||(P>=9&&P<=12)||P==23)?1:0);PINDIR(P,V);}while(0);

// for pwm, we use MAT2 as PWM cycle reference for 3 counter CT160,CT320 & CT321
#define PWM_32_1_EN(F) do{SYSAHBCLKCTRL |= (1<<10);TMR32B1MR2 = SYS_FREQ/F;TMR32B1MCR = (1<<7);\
		TMR32B1TC = 0;TMR32B1TCR = 1;}while(0);
#define PWM_32_0_EN(F) do{SYSAHBCLKCTRL |= (1<<9);TMR32B0MR2 = (SYS_FREQ/F);TMR32B0MCR = (1<<7);\
		TMR32B0TC = 0;TMR32B0TCR = 1;}while(0);
#define PWM_16_EN(F) do{SYSAHBCLKCTRL |= 1<<7;TMR16B0MR2 = SYS_FREQ/F;TMR16B0MCR = (1<<7);\
		TMR16B0TC = 0;TMR16B0TCR = 1;}while(0);
#define pwm_enable(C,F) {\
		SYSAHBCLKCTRL |= BIT6 + BIT13 + BIT16;\
		if(C==PWM0){\
			PWM_32_0_EN(F);\
		}else if(C==PWM1){\
			PWM_32_1_EN(F);\
		}else{PWM_16_EN(F);}\
	}
#define pwm_write(P,D){\
		if(P==1){IOFUN_SET(P,0x2);TMR16B0PWMC|=BIT0;TMR16B0MR0=(100-D)*TMR16B0MR2/100;}else \
			if(P==2){IOFUN_SET(P,0x2);TMR16B0PWMC|=BIT1;TMR16B0MR1=(100-D)*TMR16B0MR2/100;} else \
				if(P==4){IOFUN_SET(P,0x3);TMR32B0PWMC|=BIT3;TMR32B0MR3=(100-D)*TMR32B0MR2/100;}else \
					if(P==15){IOFUN_SET(P,0x2);TMR32B0PWMC|=BIT0;TMR32B0MR0=(100-D)*TMR32B0MR2/100;}else \
						if(P==16){IOFUN_SET(P,0x2);TMR32B0PWMC|=BIT1;TMR32B0MR1=(100-D)*TMR32B0MR2/100;}else \
							if(P==10){IOFUN_SET(P,0x3);TMR32B1PWMC|=BIT0;TMR32B1MR0=(100-D)*TMR32B1MR2/100;}else \
								if(P==11){IOFUN_SET(P,0x3);TMR32B1PWMC|=BIT1;TMR32B1MR1=(100-D)*TMR32B1MR2/100;} else \
									if(P==13){IOFUN_SET(P,0x2);TMR32B1PWMC|=BIT3;TMR32B1MR3=(100-D)*TMR32B1MR2/100;}\
	}
	//#define gpio_pin_mode(P,V,M) (M==DIGITAL?gpio_pin_mode(P,V):ANALOG_IO_EN(P,V))
