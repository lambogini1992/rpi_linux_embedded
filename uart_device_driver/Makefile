obj-m += dev_serial.o

all:
	make -C /home/anhpt26/rpi-tools/linux M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules
	dtc -I dts -O dtb -f mcu_uart_overlays.dts -o mcu_uart_overlays.dtbo
clean:
	make -C /home/anhpt26/rpi-tools/linux M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean
	rm -rf mcu_uart_overlays.dtbo
