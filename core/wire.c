#include "wire.h"
void PINCONF(int P,void (*F)(unsigned int volatile*, int),int V) {
	P==1?F(&IOCON_PIO0_8,V):(P==2?F(&IOCON_PIO0_9,V):(P==3?F(&IOCON_SWCLK_PIO0_10,V):(P==4?F(&IOCON_R_PIO0_11,V):(
	P==5?F(&IOCON_PIO0_5,V):(P==6?F(&IOCON_PIO0_6,V):(P==9?F(&IOCON_R_PIO1_0,V):(P==10?F(&IOCON_R_PIO1_1,V):(P==11?F(&IOCON_R_PIO1_2,V):(
	P==12?F(&IOCON_SWDIO_PIO1_3,V):(P==13?F(&IOCON_PIO1_4,V):(P==14?F(&IOCON_PIO1_5,V):(P==15?F(&IOCON_PIO1_6,V):(P==16?F(&IOCON_PIO1_7,V):(
	P==17?F(&IOCON_PIO1_8,V):(P==18?F(&IOCON_PIO1_9,V):(P==23?F(&IOCON_RESET_PIO0_0,V):(P==24?F(&IOCON_PIO0_1,V):(P==25?F(&IOCON_PIO0_2,V):(
	P==26?F(&IOCON_PIO0_3,V):(P==27?F(&IOCON_PIO0_4,V):(P==28?F(&IOCON_PIO0_7,V):0xFFF)))))))))))))))))))));}

int BIT_S(int P) {
	return (P==1?8:(P==2?9:(P==3?10:(P==4?11:(P==5?5:(P==6?6:(P==9?0:(P==10?1:(P==11?2:(
	P==12?3:(P==13?4:(P==14?5:(P==15?6:(P==16?7:( P==17?8:(P==18?9:(P==23?0:(P==24?1:(P==25?2:(
	P==26?3:(P==27?4:(P==28?7:12))))))))))))))))))))));
}

void RSET(unsigned int volatile* R,int V) {*R |= V;}
void RCLEAR(unsigned int volatile *R,int V) {*R &= (~V);}
void IOFUN_SET(int P,int V) {PINCONF(P,RCLEAR,0x7);PINCONF(P,RSET,V);}

//interrup
void (*UART_IRQ)(void) = NULL;//uart interup service
void (*SYS_TICK_IRQ)(void) = NULL;//sys_tick interrup service
void (*GPIO0_IRQ[12])(void) = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
void (*GPIO1_IRQ[10])(void) = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL}; 
void (*SPI_IRQ)(void) = NULL;

void __serial_irq(void){if(UART_IRQ != NULL){UART_IRQ();}else{while(1);}}
void __sys_tick(void){if(SYS_TICK_IRQ != NULL){SYS_TICK_IRQ();}else{while(1);}}
void __HARD_FAULT(void)
{
	serial_writenl("Hard fault");
}
void __GPIO_IRQ(void){
		
	int i = 0;
	for(i=0;i<12;i++)
	{
		if(((GPIO0RIS & (1<<i)) > 0) && (GPIO0_IRQ[i] != NULL))
		{
			GPIO0_IRQ[i]();
			GPIO0IC = (1<<i);
			//ICPR |= BIT31;// clear pending interupt
		}
	}
	for(i=0;i<10;i++)
	{
		if(((GPIO1RIS & (1<<i)) > 0) && (GPIO1_IRQ[i] != NULL))
		{
			GPIO1_IRQ[i]();
			GPIO1IC = (1<<i);
		}
	}
}
void __spi_irq(void){
	if(SPI_IRQ!= NULL){
		//disable_interrupts();
		SSP0IMSC &= ~BIT1;
		SPI_IRQ();
		// remove remaining unused data to quit the irq
		uint8 dummy = 0;
		// TX
		while ( (SSP0SR & (BIT1)) == BIT1 )
		 	SSP0DR = dummy;
		 //RX
		while((SSP0SR & BIT2)== BIT2)
			dummy = SSP0DR;
		SSP0ICR |= BIT0+BIT1;
		SSP0IMSC |= BIT1;
		serial_write_int(SSP0RIS);
		//enable_interrupts();
	}else{
		while(1);
	}
}

//inline int PORT(int P) {return (P>=9&&P<=18)?0x1:0x0;}
void GPIO_DIR_SET(unsigned int volatile* R,int P,int V) {*R = (V == IN)?(*R&(~(1<<BIT_S(P)))):(*R|(1<<BIT_S(P)));}
void PINDIR(int P, int V) {(PORT(P) == 0x1)?GPIO_DIR_SET(&GPIO1DIR,P,V):GPIO_DIR_SET(&GPIO0DIR,P,V);}

int GPIO_READ(unsigned int volatile* R,int P) {return *R & (1<<BIT_S(P))?HIGH:LOW;}
void GPIO_DATA_SET(unsigned int volatile* R,int P,int V) {*R = (V == HIGH)?(*R|(1<<BIT_S(P))):(*R&(~(1<<BIT_S(P))));}

void digital_write(int P, int V) {(PORT(P) == 0x1)?GPIO_DATA_SET(&GPIO1DATA,P,V):GPIO_DATA_SET(&GPIO0DATA,P,V);}
int digital_read(int P) {return (PORT(P) == 0x1)?GPIO_READ(&GPIO1DATA,P):GPIO_READ(&GPIO0DATA,P);}

void gpio_pin_mode(int P,int V) {
	SYSAHBCLKCTRL |= BIT6  + BIT16;
	IOFUN_SET(P,(P==3||P==4||(P>=9&&P<=12)||P==23)?1:0);
	if(P==5 || P==27){PINCONF(P,RCLEAR,BIT8+BIT9);PINCONF(P,RSET,BIT8);}
	//enable puldown
	PINCONF(P,RCLEAR,BIT3+BIT4);
	PINCONF(P,RSET,BIT3);
	PINDIR(P,V);
}
void gpio_interrupt(int p,int mode,void (*func)())
{
	//setup the interrup
	gpio_pin_mode(p,IN);
	if(PORT(p)==0)
	{
		GPIO0IS &= ~(1<<BIT_S(p));
		GPIO0IBE &= ~(1<<BIT_S(p));
		GPIO0IEV = mode == RISING? (GPIO0IEV | (1<<BIT_S(p))):(GPIO0IEV & ~(1<<BIT_S(p)));
		GPIO0IC = (1<<BIT_S(p));
		GPIO0IE |= (1<<BIT_S(p));
		ISER |= BIT31;
		GPIO0_IRQ[BIT_S(p)] = func;
	}
	else
	{
		GPIO1IS &= ~(1<<BIT_S(p));
		GPIO1IBE &= ~(1<<BIT_S(p));
		GPIO1IEV = mode == RISING? (GPIO1IEV | (1<<BIT_S(p))):(GPIO1IEV & ~(1<<BIT_S(p)));
		GPIO1IC = (1<<BIT_S(p));
		GPIO1IE |= (1<<BIT_S(p));
		ISER |= BIT30;
		GPIO1_IRQ[BIT_S(p)] = func;
	}
	enable_interrupts();
}
void pwm_enable(int C,int F) {
		SYSAHBCLKCTRL |= BIT6 + BIT13 + BIT16;
		if(C==PWM0){
			SYSAHBCLKCTRL |= (1<<9);
			TMR32B0MR2 = (SYS_FREQ/F);
			TMR32B0MCR = (1<<7);
			TMR32B0TC = 0;TMR32B0TCR = 1;
		}else if(C==PWM1){
			SYSAHBCLKCTRL |= (1<<10);
			TMR32B1MR2 = SYS_FREQ/F;
			TMR32B1MCR = (1<<7);
			TMR32B1TC = 0;
			TMR32B1TCR = 1;
		}else{
			SYSAHBCLKCTRL |= 1<<7;
			TMR16B0MR2 = SYS_FREQ/F;
			TMR16B0MCR = (1<<7);
			TMR16B0TC = 0;
			TMR16B0TCR = 1;
		}
	}
void pwm_write(int P,int D){
		if(P==1){IOFUN_SET(P,0x2);TMR16B0PWMC|=BIT0;TMR16B0MR0=(100-D)*TMR16B0MR2/100;}else 
			if(P==2){IOFUN_SET(P,0x2);TMR16B0PWMC|=BIT1;TMR16B0MR1=(100-D)*TMR16B0MR2/100;} else 
				if(P==4){IOFUN_SET(P,0x3);TMR32B0PWMC|=BIT3;TMR32B0MR3=(100-D)*TMR32B0MR2/100;}else 
					if(P==15){IOFUN_SET(P,0x2);TMR32B0PWMC|=BIT0;TMR32B0MR0=(100-D)*TMR32B0MR2/100;}else 
						if(P==16){IOFUN_SET(P,0x2);TMR32B0PWMC|=BIT1;TMR32B0MR1=(100-D)*TMR32B0MR2/100;}else 
							if(P==10){IOFUN_SET(P,0x3);TMR32B1PWMC|=BIT0;TMR32B1MR0=(100-D)*TMR32B1MR2/100;}else 
								if(P==11){IOFUN_SET(P,0x3);TMR32B1PWMC|=BIT1;TMR32B1MR1=(100-D)*TMR32B1MR2/100;} else 
									if(P==13){IOFUN_SET(P,0x2);TMR32B1PWMC|=BIT3;TMR32B1MR3=(100-D)*TMR32B1MR2/100;}
}
void serial_begin(int baut)
{
	int count;
	SYSAHBCLKCTRL |= BIT6 + BIT16;
	IOFUN_SET(15,0x1);
	IOFUN_SET(16,0x1);
	SYSAHBCLKCTRL |= (1<<12); //enable clock to Uart 
	
	UARTCLKDIV |= 1;//PCLK=48Mhz
	U0LCR |= BIT7;//enable divisor latch
	U0FDR = (1<<4)+0;//DIVADDVAL = 0 and MULVAL=1
	count = SYS_FREQ/(16*baut) ;
	U0DLM = count/256;
	U0DLL = count%256;
	U0LCR &= ~BIT7;//disable divisor latch
	
	U0FCR |= 0x01; //enable the FIFO
	U0LCR |= 0x03; // 8 bit data, 1 stop bit, no parity
	U0TER |= 0x80;	
}
 void serial_write_byte(uint8 data)
 {
	 while(!(U0LSR & 0x20));//data is busy??
	 U0THR |= data;
 }
 void serial_write(const char* str)
 {
	 //disable_interrupts();
	 while(*str != '\0')
	 {
	 	serial_write_byte(*str);
		str++;
	 }
	 //serial_write_byte('\0');
	 //enable_interrupts();
 }
 void serial_writenl(const char* str)
 {
	 serial_write(str);
	 serial_write("\r\n");
 }
 void serial_write_buff(uint8* data, int nbyte, bool HEX)
 {
	 int i=0;
	 disable_interrupts();
	 for(i=0;i<nbyte;i++)
	 {
		 if(HEX==TRUE)
		 {
			serial_write_hex(data[i]);
		 }
		 else
		 {
			 serial_write_byte(data[i]);
		 }
			 
	 }
	 enable_interrupts();
 }
 void  serial_write_int(int data)
 {
	 int rem,res;
	 if(data < 10)
	 {
	 	serial_write_byte((uint8)('0'+data));
	 }
	 else
	 {
		 rem = data%10;
		 res = data/10;
		 serial_write_int(res);
		 serial_write_int(rem);
	 }
 }
 void serial_write_hex(uint8 data)
 {
	 uint8 upper,lower;
	 upper = data >> 4;
	 lower = data & 0x0f;
	 serial_write_byte(upper<10?(upper+'0'):((upper - 10) +'A'));
	 serial_write_byte(lower<10?(lower+'0'):((lower - 10) +'A'));
 }
 void serial_irq(void (*func)())
 {
 	U0IER = BIT0+BIT2;
 	ISER |= BIT21;//enable NVIC 13
	UART_IRQ = func;
	enable_interrupts();
 }
 int serial_available()
 {
	 return (U0LSR & 0x01);
 }
 uint8 serial_read()
 {
	 return U0RBR;
 }
 void serial_read_buff(uint8* buff,int nbyte)
 {
	 int i=0;
	 disable_interrupts();
	 while(i<nbyte)
	 {
		 if(serial_available())
		 {
			 buff[i] = serial_read();
			 i++;
		 }
	 }
	  enable_interrupts();
 }
 
 //ANALOG CONFIG
  void analog_init()
  {
  	SYSAHBCLKCTRL |= BIT6 + BIT13 + BIT16;
  	// Power up the ADC
  	PDRUNCFG &= ~BIT4;
 	// set ADC clock rate
 	AD0CR=(11<<8)+BIT16;
	// use the burst mode to enable multi adc at the same time
  }
 void analog_enable(int p)
 {  
 	// select analog mode for pin
	PINCONF(p, RCLEAR,0xFFFF);
	PINCONF(p,RSET,0x2);
 }
 int analog_read(int p)
 {
	 int chanel = p==4?BIT0:(p==9?BIT1:(p==10?BIT2:(p==11?BIT3:(p==12?BIT4:BIT5))));
	 unsigned int volatile * DR =  p==4?&AD0DR0:(p==9?&AD0DR1:(p==10?&AD0DR2:(p==11?&AD0DR3:(p==12?&AD0DR4:&AD0DR0))));
	 // Select the channel
	 AD0CR &= ~(0xFF);
	 AD0CR |=chanel;
	 // Wait for conversion to complete
	 while((*DR&BIT31)==0);
 	// return the result
 	return ((*DR>>6)&0x3ff);
 }
 void __delay_cycles(int value)
 {
	 __asm__ volatile 
   	 (
		//"sub r0,#2					\n\t"
		"__delay_us_wait:			\n\t"
		"	sub r0,#1				\n\t"
		"	bne __delay_us_wait		\n\t"
    );
}
void delay_us(int value)
{
	__delay_cycles(value*12);
}
 void delay_ms(int value)
 {
	 delay_us(value*1000);
 }
 void delay_s(int value)
 {
	 delay_ms(value*1000);
 }
 void sonar_init(int trigger, int echo)
 {
	 gpio_pin_mode(trigger,OUT);
	 gpio_pin_mode(echo,IN);
 }
 
 int sonar_get_distance(int trigger, int echo)
 {
	 int counter = 0;
	 int max = 60000; //us
	 //first reset the trigger
	 digital_write(trigger,LOW);
	 delay_us(5);
	 // trigger is high for 10us to active the sensor
	 digital_write(trigger,HIGH);
	 delay_us(10);
	 digital_write(trigger,LOW);
	 //enable the system tick, enable interrup, use system clock
	 SYST_CSR |= BIT0+BIT2;
	 SYST_RVR = SYS_FREQ/100000 - 1;
	 SYST_CVR = 5;
	 while(counter*10<max)
	 {
		 while(SYST_CVR > 5){if(digital_read(echo)==HIGH) break;}
		 if(digital_read(echo)==HIGH) break;
		 counter++;
	 }
	 //echo is High, get the pulse width
	 counter = 0;
	 SYST_CVR = 5;
	 while(digital_read(echo) == HIGH)
	 {
		 while(SYST_CVR > 5);
		 counter++;
	 }
	 SYST_CSR &= ~(BIT0+BIT2);
	 //convert to distance in cm
	 if(counter > 0)
		 return counter*10/57;
	 else
		 return -1;
 }
 int __div_and_round(int a,int b)
 {
	 int v = a/b;
	 int mod = a%b;
	 if(mod > b/2) v=v+1;
	 return v;
 }
 //servo controller
 void servo_init(int channel,int pin) //PWM0,1,2
 {
 	pwm_enable(channel,50);
 	pwm_write(pin,3);//0 degree
 }
 void servo_write(int pin,int angle)
 {
	 // duty = (0.5 + 0.01*angle)*100/20
	 // ontime for 1 degree is 0.01 ms beetween 0.5 and 2.5 coresponding to 0 and 180
	 //period is 20 ms
	 int duty = __div_and_round((50 + angle),20);
	 pwm_write(pin,duty);
 }
 
 void spi_init(int mode)
 {
	 uint8 dummy = 0,i;
	 PRESETCTRL &= ~(1<<0);
 	PRESETCTRL |= (1<<0);//spi0 reset de-assert
	SYSAHBCLKCTRL |= BIT11 + BIT6+  BIT16;//enable clock for spi @ iocon
	SSP0CLKDIV  = 0x01;//divide par 1
	
	IOFUN_SET(1,0x1);//select MISO0
	IOFUN_SET(2,0x1);// select MOSI0
	IOCON_SCK0_LOC = 0x2; //select pin 6 for sclk
	IOFUN_SET(6,0x2);//select SCLK0
	//configure SPI pin
	if(mode == SLAVE)
	{
		//use chip select, the device is selected when this is high
		IOFUN_SET(25,0x1);
		SSP0CR1 &= ~BIT1;
		SSP0CR1 |= BIT2;
		SSP0CR1 |= BIT1;
	}
	else
	{
		//user pin25 as chip select to device
		gpio_pin_mode(25,OUT);
		digital_write(25,HIGH);
		SSP0CR1 &= ~BIT2;
		//configure for master here
	}
	//loop back for test
	//SSP0CR1 |= BIT0;
	//select 8bit spi
	SSP0CR0 =  0x7 ;
	// clock prescale
	SSP0CPSR = 0x2;
	//the fifo size is 8, let empty it
	for(i=0;i<8;i++){dummy = SSP0DR;}
 }
 void spi_send(uint8* data, int length)
 {
	 int i;
	 uint8 dummy = 0;
	 for ( i = 0; i < length; i++ )
	 {
		 /* Move on only if NOT busy and TX FIFO not full. */
		 while ( (SSP0SR & (BIT1)) != BIT1 );
		 SSP0DR = *data;
		 data++;

		 while ( (SSP0SR & (BIT2+BIT4)) != BIT2 );//BIT2+BIT4
		/* Whenever a byte is written, MISO FIFO counter increments, Clear FIFO 
		on MISO. Otherwise, when SSP0Receive() is called, previous data byte
		is left in the FIFO. */
		dummy = SSP0DR;
	  }
	  return; 
 }
 void spi_data_from_fifo(uint8* buff, int size)
 {
 	/*this function is used only for interrupt reception*/
 	 int i;
	 for ( i = 0; i < size; i++ )
	 {
		//SSP0DR = 0xFF;
	 	while ( (SSP0SR & BIT2) != BIT2 );
	 	*buff = SSP0DR;
		buff++;
	 }
 }
 void spi_receive(uint8* buff, int size)
 {
	 int i;
	 for ( i = 0; i < size; i++ )
	 {
	 	/* As long as Receive FIFO is not empty, I can always receive. */
	 	/* If it's a loopback test, clock is shared for both TX and RX,
	 	no need to write dummy byte to get clock to get the data */
	 	/* if it's a peer-to-peer communication, SSPDR needs to be written
	 	before a read can take place. */
	 	/* Wait until the Busy bit is cleared */
		SSP0DR = 0xFF;
	 	while ( (SSP0SR & (BIT2+BIT4)) != BIT2 );
	 	*buff = SSP0DR;
		//while ( (SSP0SR & (BIT1)) != BIT1 );
	 	//SSP0DR = *buff;
		buff++;
		/*send back dummy byte*/
		
	 }
	   return; 
 }
 void spi_irq(void (*func)())
 {
 	SSP0IMSC |= BIT1;
 	ISER |= BIT20;//enable NVIC 13
	SPI_IRQ = func;
	enable_interrupts();
 }
