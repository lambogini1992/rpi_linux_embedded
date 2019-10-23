 For compiling new device tree overlay, we can use like compiling dtb file:

 	Ex: dtc -I dts -O dtb -f mcu_uart_overlays.dts -o mcu_uart_overlays.dtbo 

 For loading device tree overlay(*.dtbo) into linux system board. We need to use command on linux:
 
	+ dtoverlay=*.dtbo

 After that, we can insmod platform device driver *.ko into system. 