obj-m += fxos8700.o
LINUX_SOURCE = /home/oem/imx6-udoo-board/linux_kernel
AR = arm

all:
	make -C $(LINUX_SOURCE) M=$(shell pwd) ARCH=$(AR) CROSS_COMPILE=arm-linux-gnueabihf- modules

clean:
	make -C $(LINUX_SOURCE) M=$(shell pwd) ARCH=$(AR) CROSS_COMPILE=arm-linux-gnueabihf- clean	 