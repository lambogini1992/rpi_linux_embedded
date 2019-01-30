#ifndef __GPIO_LIB_DRIVER_H__
#define __GPIO_LIB_DRIVER_H__

#include <stdbool.h>
#include "gpio_register.h"

#define INPUT      0b000
#define OUTPUT     0b001
#define FUNC_0     0b100
#define FUNC_1     0b101
#define FUNC_2     0b110
#define FUNC_3     0b111
#define FUNC_4     0b011
#define FUNC_5     0b010
#define HIGH       1
#define LOW        0

void init_gpio(uint8_t port, uint32_t func);
void set_level_output(uint8_t port, bool level);
void read_status_gpio(uint8_t port, uint32_t *data_level);


#endif
