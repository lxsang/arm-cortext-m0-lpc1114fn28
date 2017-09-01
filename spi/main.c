#include "../core/wire.h"
uint8 data[8] ={0xAB,0xFF,0xDE,0xBF,0xDD,0xBB,0xEE};

void spi_handle()
{
	serial_writenl("SPI IRQ:");
	spi_data_from_fifo(data,8);
	serial_write_buff(data,8,TRUE);
	serial_writenl("sending");
	spi_send(data,8);
	serial_writenl("send complete");
	serial_writenl("");
	//delay_ms(100);
}

int main ()
{
	spi_init(SLAVE);
	serial_begin(19200);
	spi_irq(spi_handle);
	while(1)
	{
		delay_ms(200);
		serial_writenl("nop");
	}
}