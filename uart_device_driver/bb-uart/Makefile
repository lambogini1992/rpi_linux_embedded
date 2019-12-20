obj-m += mcu_uart.o


all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	dtc -@ -I dts -O dtb -o BB-MCU-UART-00A0.dtbo mcu_uart.dts
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f *.dtbo

