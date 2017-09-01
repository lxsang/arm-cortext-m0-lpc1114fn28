PREFIX=arm-none-eabi-
CORE_PATH=../core
LIB=/Users/mrsang/private-app/gcc-arm-none-eabi-4_9/lib/gcc/arm-none-eabi/4.9.3/armv6-m
PROJ_NAME=spi
REMOTE_DIR=root@10.200.14.85:/root/hex-repos
	
all:clean init.o  main.o wire.o
	$(PREFIX)ld init.o wire.o main.o -O1 -L $(LIB) -lgcc -T $(CORE_PATH)/linker_script.ld --cref -Map main.map -nostartfiles -o main.elf
	$(PREFIX)objcopy -O ihex main.elf $(PROJ_NAME).hex
	# sync to server
	scp $(PROJ_NAME).hex $(REMOTE_DIR)/$(PROJ_NAME).hex
init.o:
	$(PREFIX)gcc -mcpu=cortex-m0 -mthumb -O1 -g -c $(CORE_PATH)/init.c -o init.o
wire.o:
	$(PREFIX)gcc -mcpu=cortex-m0 -mthumb -O1 -g -c $(CORE_PATH)/wire.c -o wire.o
main.o:
	$(PREFIX)gcc -mcpu=cortex-m0 -mthumb -O1 -g -c main.c -o main.o
clean:
	-rm main.o  init.o main.map main.elf wire.o $(PROJ_NAME).hex