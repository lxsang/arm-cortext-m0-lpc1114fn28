#include "../core/wire.h"
void test_interrupt(void)
{
	serial_writenl("OOPS! I hit the wall");
}
int main ()
{
	serial_begin(19200);
	sonar_init(18,17);
	gpio_interrupt(28,RISING,test_interrupt);
	int dist;
	while(1)
	{
		dist = sonar_get_distance(18,17);
		serial_write("Distance: ");
		serial_write_int(dist);
		serial_write(" cm \r\n");
		delay_ms(100);
	}
}