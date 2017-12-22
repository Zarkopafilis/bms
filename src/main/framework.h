/* Extensions and additions on LT_SPI and more 
importantly LTC60842 functions to make them work for teensy. */

#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <stdint.h>

extern int pec;

void output_low(uint8_t pin);
void output_high(uint8_t pin);

//10K Thermistor on VRef2 and GPIOx Pin
float volts_to_celsius(float cell, float vref);
void shut_car_down();

//Sets up the BMS for the car being ready to drive (not charging)
void setup_drive_mode();

#endif //FRAMEWORK_H
