obj-m += mcu_uart.o

SRC_BUILD = /home/oem/beaglebone/linux
GCC = arm-linux-gnueabihf-

all:
	make -C $(SRC_BUILD) M=$(shell pwd) ARCH=arm CROSS_COMPILE=$(GCC) modules
	dtc -@ -I dts -O dtb -o mcu_uart.dtbo mcu_uart.dts

clean:
	make -C$(SRC_BUILD) M=$(shell pwd) ARCH=arm CROSS_COMPILE=$(GCC) clean
	rm -f mcu_uart.dtbo	

