obj-m += dev_serial.o

all:
	make -C /home/anhpt26/rpi-tools/linux M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules
	dtc -@ -I dts -O dtb -o mcu_uart.dtbo mcu_uart_overlays.dts
clean:
	make -C /home/anhpt26/rpi-tools/linux M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean
	rm -rf mcu_uart.dtbo
