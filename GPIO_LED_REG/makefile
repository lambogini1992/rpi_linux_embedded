SOURCE=main.c gpio_lib_driver.c gpio_register.c
GCC=arm-linux-gnueabihf-gcc
CFLAG=-I -wall

all:$(SOURCE)
	$(GCC) $(CFLAG) $(SOURCE) -o gpio_reg_test.exe

clean:
	rm -rf *.exe
