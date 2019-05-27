#ifndef _GPIO_REG_H_
#define _GPIO_REG_H_


#define BCM2837_REG_BASE			0x3F000000
#define GPIO_BASE					(BCM2837_REG_BASE + 0x200000)
#define TOTAL_GPIO_REG				0xC0

/*Function Setting Register(Input/Output/function)
*They are read and write register
*we asign 3 bit for setting function one pin
*0b000 is INPUT  for each pin
*0b001 is OUTPUT
*0b100 is take alternate function 0
*0b101 is take alternate function 1
*0b110 is take alternate function 2
*0b111 is take alternate function 3
*0b011 is take alternate function 4
*0b010 is take alternate function 5
*/
#define GPFSEL0                 0
#define GPFSEL1                 1
#define GPFSEL2                 2
#define GPFSEL3                 3
#define GPFSEL4                 4
#define GPFSEL5                 5

/*GPIO Pin Output Set Register
*They are written register
*we asign 1 bit to set output level for each pin
*0 is not effect to OUTPUT pin
*1 is setting high level
*/
#define GPSET0                 7
#define GPSET1                 8

/*GPIO Pin Output Clear Register
*They are written register
*we asign 1 bit to set output level for each pin
*0 is not effect to OUTPUT pin
*1 is setting low level
*/
#define GPCLR0                 10
#define GPCLR1                 11

/*GPIO Pin Level
*They are Readed register
*we asign 1 bit to set output level for each pin
*They are used to read status level of INPUT PIN
*/
#define GPLEV0                 13
#define GPLEV1                 14

/*Start of Register Support for Interrupt*/

/*GPIO Pin Event Detect Status
*They are Readed and Written register
*we asign 1 bit to set output level for each pin
*They are used to record status of PIN.
*If GPIO pin change status. It will set to 1.
*It is clear by manual. We must set bit to 0 if we want to clear it
*/
#define GPEDS0                 16
#define GPEDS1                 17

/*GPIO Pin Rising Edge Detect Enable
*They are Readed and Written register
*we asign 1 bit to set output level for each pin
*They are used to detect rising signal of PIN.
*If GPIO pin detect rising edge status. It will set to 1.
*It is clear by manual. We must set bit to 0 if we want to clear it
*/
#define GPREN0                 19
#define GPREN1                 20

/*GPIO Pin Falling Edge Detect Enable
*They are Readed and Written register
*we asign 1 bit to set output level for each pin
*They are used to detect falling signal of PIN.
*If GPIO pin detect Falling Edge status. It will set to 1.
*It is clear by manual. We must set bit to 0 if we want to clear it
*/
#define GPFEN0                 22
#define GPFEN1                 23

/*GPIO Pin High Detect Enable
*They are Readed and Written register
*we asign 1 bit to set output level for each pin
*They are used to detect GPIO PIN is High Level status
*If GPIO pin detect at High status. It will set to 1.
*It is clear by manual. We must set bit to 0 if we want to clear it
*/
#define GPHEN0                 25
#define GPHEN1                 26

/*GPIO Pin Low Detect Enable
*They are Readed and Written register
*we asign 1 bit to set output level for each pin
*They are used to detect GPIO PIN is Low Level status
*If GPIO pin detect at Low status. It will set to 1.
*It is clear by manual. We must set bit to 0 if we want to clear it
*/
#define GPLEN0                 28
#define GPLEN1                 29

/*GPIO Pin Async. Rising Edge Detect
*They are Readed and Written register
*we asign 1 bit to set output level for each pin
*They are used to record status of PIN.
*If GPIO pin detect rising Edge status, not depend on system clock. It will set to 1.
*It is clear by manual. We must set bit to 0 if we want to clear it
*/
#define GPAREN0               31
#define GPAREN1               32

/*GPIO Pin Async. Falling Edge Detect
*They are Readed and Written register
*we asign 1 bit to set output level for each pin
*They are used to record status of PIN.
*If GPIO pin detect Falling Edge status, not depend on system clock. It will set to 1.
*It is clear by manual. We must set bit to 0 if we want to clear it
*/
#define GPAFEN0               34
#define GPAFEN1               35


/*End of Register Support for Interrupt*/


/*GPIO Pull-up/down Register
*They are Readed and Written register
*We just use first 2 bit of GPPUD register
*It is used controls the actuation of the internal pull-up/down control line to ALL the GPIO pins
*0b00 is disable Pull-Up/Down
*0B01 is Enable Pull Down
*0b10 is Enable Pull Up
*/
#define GPPUD                  37

/*GPIO Pin Pull-up/down Enable Clock
*They are Readed and Written register
*we asign 1 bit to set output level for each pin
*They are used to record status of PIN.
*If GPIO pin change status. It will set to 1.
*It is clear by manual. We must set bit to 0 if we want to clear it
*/
#define GPPUDCLK0               38
#define GPPUDCLK1               39

	

#endif