#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/timer.h>
#include "gpio_reg_kernel.h"

#define INPUT      0b000
#define OUTPUT     0b001

void init_gpio(uint8_t port, uint32_t func)
{
    uint8_t check_reg;
    uint8_t port_reg;

    check_reg = port/10;
    port_reg  = port%10;

    switch (check_reg)
    {
      case 0:
      GPFSEL0 |= func << port_reg;

      case 1:
      GPFSEL1 |= func << port_reg;

      case 2:
      GPFSEL2 |= func << port_reg;

      case 3:
      GPFSEL3 |= func << port_reg;

      case 4:
      GPFSEL4 |= func << port_reg;

      case 5:
      GPFSEL5 |= func << port_reg;
    }
}

void set_level_output(uint8_t port, bool level)
{
  uint8_t temp_port;

  if(1 == level)
  {
    if(32 > port)
    {
      GPSET0 |= 1 << port;
    }
    else
    {
      temp_port = port - 32;
      GPSET1 |= 1 << temp_port;
    }
  }
  else
  {
    if(32 > port)
    {
      GPCLR0 |= 1 << port;
    }
    else
    {
      temp_port = port - 32;
      GPCLR1 |= 1 << temp_port;
    }
  }
}

static int __init gpio_kernel(void)
{

    gpio = (volatile unsigned int *)ioremap(GPIO_BASE_PERIPHERAL, TOTAL_GPIO_REG);
    init_gpio((uint8_t)17,(uint32_t)OUTPUT);
    set_level_output((uint8_t)17,1);
    printk(KERN_INFO "Hello world!\n");
    return 0;    // Non-zero return means that the module couldn't be loaded.
}

static void __exit remove_kernel(void)
{
    set_level_output(17,0);
    printk(KERN_INFO "Cleaning up module.\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("PTA");
MODULE_DESCRIPTION("A GPIO module");

module_init(gpio_kernel);
module_exit(remove_kernel);
