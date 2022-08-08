export ARCH ?= arm
export CROSS_COMPILE ?= arm-linux-gnueabihf-

obj-m += lcd_i2c.o

KERNEL_DIR ?= ../linux

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean