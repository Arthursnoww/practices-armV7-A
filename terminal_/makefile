CROSS_COMPILE = arm-linux-gnueabihf-

all2: startup.s
	$(CROSS_COMPILE)as *.s -o startup.o
	$(CROSS_COMPILE)ld -o startup -T memmap startup.o
	$(CROSS_COMPILE)objcopy startup b3.bin -O binary
	$(CROSS_COMPILE)objdump -DSx -b binary -marm b3.bin > startup.lst
	cp *.bin /tftpboot


clean:
	rm *.o *.bin *.lst
