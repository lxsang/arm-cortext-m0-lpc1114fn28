
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cpu.h>

#define BUF_SIZE 64
#define CMD_SIZE 3

uint8_t data_buffer[BUF_SIZE];
uint8_t cmd[CMD_SIZE];

void SPI0_IRQHandler(void)
{
    uint8_t *p;
    int i;
    p = (uint8_t *) &cmd;

    for (i = 0; i < CMD_SIZE; i++)
    {
        while (!(LPC_SSP0->SR & 0x04));
        *p++ = LPC_SSP0->DR;
    }
    
    switch(cmd[0])
    {
        case 0: // read by index
            while (!(LPC_SSP0->SR & 0x02));
            LPC_SSP0->DR = data_buffer[cmd[1]];
            break;
        case 1: // write by index
            data_buffer[cmd[1]] = cmd[2];
            break;
        case 2: // read all data 
            for(i = cmd[1]; i < cmd[2];i++)
            {
                while (!(LPC_SSP0->SR & 0x02));
                LPC_SSP0->DR = data_buffer[i];
            }
    }

    // Drain the receive FIFO

    while (LPC_SSP0->SR & 0x04) (void) LPC_SSP0->DR;
}

int main(void)
{
  
    char buf[10];
    int status,i;
    cpu_init(DEFAULT_CPU_FREQ);
    serial_stdio(CONSOLE_PORT);
    //init spi
    if ((status = spi_slave_init(0, 8, 3, SPI_MSBFIRST)))
    {
        printf("ERROR: spi_master_init() failed at line %d, %s\n", status, strerror(errno));
        exit(1);
    }
    
    NVIC_EnableIRQ(SSP0_IRQn);
    // enable interrupt
    LPC_SSP0->IMSC |= (1<<1);
    
    for (;;)
    {
        printf("Enter command: ");
    
        gets(buf);
        switch(buf[0])
        {
            case 'd': // data 
                for(i = 0; i < BUF_SIZE; i++)
                {
                    printf("%d ", data_buffer[i]);
                    if((i+1)%8 == 0)
                        printf("\n");
                }
                break;
            case 'c': // clear 
                for(i = 0; i < BUF_SIZE;i++) data_buffer[i] = 0;
                break;
            default:
                printf("Unknow command. Enter c (clear) or d (display)");
        }
    }
}
