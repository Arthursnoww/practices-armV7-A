CHAIN=$(CHAINPATH)arm-none-eabi-

all: startup.s
	$(CHAIN)as startup.s -o lab1.o
	$(CHAIN)objcopy lab1.o lab1.bin -O binary
	$(CHAIN)objdump -D -b binary -marm lab1.bin > lab1.lst
	cp lab1.bin /tftpboot

clean:
	rm *.o *.bin *.lst