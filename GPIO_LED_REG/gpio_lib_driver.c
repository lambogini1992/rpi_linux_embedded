#include "gpio_lib_driver.h"

// static bool set_up_io_var = FALSE;

void init_gpio(uint8_t port, uint32_t func)
{
    uint8_t check_reg;
    uint8_t port_reg;

    // if(FALSE == set_up_io_var)
    // {
    //   printf("Setting up for IO memory\n");
    //   setup_io();
    //   set_up_io_var == TRUE;
    // }

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

  if(HIGH == level)
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

void read_status_gpio(uint8_t port, uint32_t *data_level)
{
  uint8_t temp_port;

  if(32 > port)
  {
    *data_level |= GPLEV0 >> port;
  }
  else
  {
    temp_port = port - 32;
    *data_level |= GPLEV1 >> port;
  }
}
