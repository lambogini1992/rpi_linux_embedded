#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"

using namespace std;

int main(int argc, char const *argv[])
{
  string data_button;
  bool k = 0;
  /* code */
  GPIOClass *gpio18 = new GPIOClass("18");
  GPIOClass *gpio4  = new GPIOClass("4");

  gpio4->setdir_gpio("in");
  gpio18->setdir_gpio("out");
  while (1) {
    gpio4->getval_gpio(data_button);

    if(data_button == "0")
    {
      usleep(200000);
      if(data_button == "0")
      {
        if(k == 0)
        {
          k = 1;
        }
        else
        {
          k = 0;
        }
      }
    }
    std::cout << k << '\n';
    if(k == 0)
    {
      gpio18->setval_gpio("0");
    }
    else
    {
      gpio18->setval_gpio("1");
    }
  }

  return 0;
}
