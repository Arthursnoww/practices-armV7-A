CROSS_COMPILE = arm-linux-gnueabihf-

#FILES = uart.s utils.s wdt.s cp15.s gpio.s

OBJS = startup.o utils.o wdt.o cp15.o gpio.o rtc.o uart.o string.o prompt.o

%.o: %.s
	$(CROSS_COMPILE)as $< -o $@


all:  $(OBJS)
	$(CROSS_COMPILE)ld -o startup -T memmap $(OBJS)
	$(CROSS_COMPILE)objcopy startup b4.bin -O binary
	$(CROSS_COMPILE)objdump -DSx -b binary -marm b4.bin > startup.lst
	cp *.bin /tftpboot

clean:
	rm *.o *.bin *.lst
	


