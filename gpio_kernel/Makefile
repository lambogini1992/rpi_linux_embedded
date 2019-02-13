obj-m += gpio_kerne.o

all:
	make -C /home/anhpt26/tool_rpi/linux/ M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules

clean:
	make -C /home/anhpt26/tool_rpi/linux/ M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean
