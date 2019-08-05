obj-m += ex_pl_dr.o
HOME = /home/anhpt26/rpi-tools/linux
all:
	make -C ${HOME} M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules
	dtc -I dts -O dtb -f bcm2709-rpi-2-b.dts -o bcm2709-rpi-2-b.dtb

clean:
	make -C ${HOME} M=$(shell pwd) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean
