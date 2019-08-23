obj-m += gpio_dev.o

all:
	make -C /home/anhpt26/rpi-tools/linux M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules
	dtc -I dts -O dtb -f bcm2709-rpi-2-b.dts -o bcm2709-rpi-2-b.dtb
clean:
	make -C /home/anhpt26/rpi-tools/linux M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean
	rm bcm2709-rpi-2-b.dtb
