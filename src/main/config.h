#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <SPI.h>
#include "LTC68042.h"

#define DEBUG 1
#define DEBUG_PEC 1
#define DEBUG_CELL_VALUES 1
#define DEBUG_TEMP_VALUES 1

//Only takes effect when DEBUG = 1
#define MEASURE_CYCLE_DEBUG_DELAY_MS 100

#define SERIAL_PORT 9600

#define SPI_MODE SPI_MODE3
#define SPI_CLOCK_DIV SPI_CLOCK_DIV16

//This is the configuration that will be written to every slave
//REFON=1 -> Always awake
//VUV (Undervoltage) & VOV (Overvoltage) Values
extern uint8_t slave_cfg[6];

//Cell discharge permitted
//Disabled = 0 | Enabled = 1
extern uint8_t DCP_MODE;

//Number of LTC6811-2 Multicell battery monitors
extern uint8_t SLAVE_NUM;

extern uint16_t MAX_MEASURE_CYCLE_DURATION_MS;

//Cells with index > CELL_IGNORE_INDEX will be ignored from measurements.
//This is useful for cases where your cells on a module 
//are < 12 (max number that the Battery Monitors can read)
//Thus, you should ignore some. In this case we have got
//each monitor observing 10 cells, thus the 11th and 12th 
//are going to be ignored both from the open wire checks and
//the measurements
//Set to 12(Max Cells) in order to measure all
extern uint8_t CELL_IGNORE_INDEX_START;
extern uint8_t CELL_IGNORE_INDEX_END;

//Same with CELL_IGNORE_INDEX, but for the GPIOs where the temperature
//sensors (or thermistors) are wired in. The monitors have 5 GPIOs
//Set to 5(Max GPIOs) in order to measure all
extern uint8_t GPIO_IGNORE_INDEX_START;
extern uint8_t GPIO_IGNORE_INDEX_END;

//Temperature Voltage Read for Undertemping and Overtemping
extern uint16_t TUT;
extern uint16_t TOT;

//Voltage for Undervolting and Overvolting
extern uint16_t VUV;
extern uint16_t VOV;

//Teensy 3.1 & 3.2 has got 2048 bytes available
#define FLAG_ADDR 0
#define CORRECT_FLAG 0xFF

#define DCP_MODE_ADDR 1
#define SLAVE_NUM_ADDR 2
//Note MAX_MEASURE_CYCLE_DURATION_MS IS uint16 | 2 bytes 
#define MAX_MEASURE_CYCLE_DURATION_MS_ADDR 3
#define CELL_IGNORE_INDEX_START_ADDR 4
#define CELL_IGNORE_INDEX_END_ADDR 5
#define GPIO_IGNORE_INDEX_START_ADDR 6
#define GPIO_IGNORE_INDEX_END_ADDR 7
//Note TUT, TOT, VUV & VOV are uint16 | 2 bytes each
#define TUT_ADDR 8
#define TOT_ADDR 10
#define VUV_ADDR 12
#define VOV_ADDR 14
//uint8_t slave_cfg[6] | 6 bytes
#define SLAVE_CONFIG_ADDR 16

#define SHUTDOWN_PIN 10
#define SHUTDOWN_PIN_IDLE 1

//Loads config from EEPROM if persist flag is correct
void config_load();

//Persists current config to EEPROM and sets persist flag 
void config_persist();

//Writes the whole config to serial if DEBUG is 1
void config_print();

#endif //CONFIG_H