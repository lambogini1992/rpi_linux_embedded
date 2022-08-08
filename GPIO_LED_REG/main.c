#include "gpio_lib_driver.h"

int main(int argc, char const *argv[])
{
  uint8_t i;
  setup_io();

  init_gpio((uint8_t)18,(uint32_t)OUTPUT);
  for ( i = 0; i < 30; i++)
  {
    set_level_output(18, HIGH);
    sleep(1);
    set_level_output(18, LOW);
    sleep(1);
  }
}
