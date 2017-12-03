/* Extensions and additions on LT_SPI and more 
importantly LTC60842 functions to make them work for teensy. */

#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <stdint.h>

// int8_t LTC6804_adow(uint8_t nIC, uint8_t ignore_index);

int8_t LTC68042_ovuv(uint8_t nIC, uint8_t ovuv[][12][2]);

#endif //FRAMEWORK_H