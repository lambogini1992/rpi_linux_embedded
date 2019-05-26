#ifndef _GPIO_REG_H_
#define _GPIO_REG_H_


#define BCM2837_REG_BASE			0x3F000000
#define GPIO_BASE					(BCM2837_REG_BASE + 0x20000)
#define TOTAL_GPIO_REG				40

/*register gpio*/
/*REG set function of gpio pin*/
#define GPFSEL0						0x00
#define GPFSEL1						0x04
#define GPFSEL2						0x08
#define GPFSEL3						0x0C
#define GPFSEL4						0x10
#define GPFSEL5						0x14

/*Set ouput level for gpio to 1. If each bit in the regs is set, corresponding bit in GPCLR is clear*/
#define GPSET0						0x1C
#define GPSET1						0x20

/*Set ouput level for gpio to 0. If each bit in the regs is set, corresponding bit in GPSET is clear*/
#define	GPCLR0						0x28
#define	GPCLR1						0x2C

/*Read current level of pin*/
#define	GPLEV0						0x34
#define	GPLEV1						0x38

/*DETECT state change of pin*/
#define GPEDS0						0x40
#define GPEDS1						0x44

/*Detect Rising edge change of pin*/
#define GPREN0						0x4C
#define GPREN1						0x50

/*Detect Falling edge change of pin*/
#define GPFEN0						0x58
#define GPFEN1						0x5C
	

#endif