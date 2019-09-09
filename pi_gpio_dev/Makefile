obj-m +=gpio_dev.o

all:
	make -C /home/anhpt26/rpi-tools/linux M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules

clean:
	make -C /home/anhpt26/rpi-tools/linux M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean
