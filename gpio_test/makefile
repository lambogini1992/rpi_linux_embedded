CC = arm-linux-gnueabihf-g++
CFLAG = -I -wall
MSRC = main.cpp
SUBSRC = gpio.cpp

build: $(MSRC) $(SUBSRC)
	$(CC) $(CFLAG) $(MSRC) $(SUBSRC) -o main.exe

# gpio.o: $(SUBSRC)
# 	$(CC) $(CFLAG) $(SUBSRC) -o gpio.o

clean:
	rm -rf *.o *.exe
