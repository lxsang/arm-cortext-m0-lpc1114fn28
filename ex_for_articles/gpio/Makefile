LD=arm-none-eabi-ld
OBJCP=arm-none-eabi-objcopy
GCC=arm-none-eabi-gcc
LIB=/Users/mrsang/private-app/gcc-arm-none-eabi-4_9/lib/gcc/arm-none-eabi/4.9.3/armv6-m
all:init.o main.o
	$(LD) init.o main.o -L $(LIB) -lgcc -T linker_script.ld --cref -Map main.map -nostartfiles -o main.elf
	$(OBJCP) -O ihex main.elf main.hex
init.o:
	$(GCC) -mcpu=cortex-m0 -mthumb -g -c init.c -o init.o
main.o:
	$(GCC) -mcpu=cortex-m0 -mthumb -g -c main.c -o main.o
clean:
	 rm main.o init.o main.map main.elf main.hex