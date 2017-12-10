#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <SPI.h>

#define DEBUG 1
#define DEBUG_PEC 1
#define DEBUG_CELL_VALUES 1
#define DEBUG_TEMP_VALUES 1

#define SERIAL_PORT 9600

#define SPI_MODE SPI_MODE3
#define SPI_CLOCK_DIV SPI_CLOCK_DIV16

//This is the configuration that will be written to every slave
//REFON=1 -> Always awake
//VUV (Undervoltage) & VOV (Overvoltage) Values
uint8_t slave_cfg[6] = 
{
 0B11111100, //GPIO 1~5 REFON DTEN ADCOPT
 0B00000101, //VUV[7~0]
 0B01100000, //VOV[3~0] VOV[11~8]
 0B00000000, //VOV[11~4]
 0B00000000, //DCC 8~1
 0B00000000 //DCTO[3~0] DCC 12~9
};

//Cell discharge permitted
//Disabled = 0 | Enabled = 1
uint8_t DCP_MODE = DCP_DISABLED

//Number of LTC6811-2 Multicell battery monitors
uint8_t SLAVE_NUM = 1

uint8_t MAX_MEASURE_CYCLE_DURATION_MS = 500

//Cells with index > CELL_IGNORE_INDEX will be ignored from measurements.
//This is useful for cases where your cells on a module 
//are < 12 (max number that the Battery Monitors can read)
//Thus, you should ignore some. In this case we have got
//each monitor observing 10 cells, thus the 11th and 12th 
//are going to be ignored both from the open wire checks and
//the measurements
//Set to 12(Max Cells) in order to measure all
uint8_t CELL_IGNORE_INDEX_START = 2
uint8_t CELL_IGNORE_INDEX_END = 12

//Same with CELL_IGNORE_INDEX, but for the GPIOs where the temperature
//sensors (or thermistors) are wired in. The monitors have 5 GPIOs
//Set to 5(Max GPIOs) in order to measure all
uint8_t GPIO_IGNORE_INDEX_START = 0
uint8_t GPIO_IGNORE_INDEX_END = 5

//Temperature Voltage Read for Undertemping and Overtemping
uint8_t TUT -10
uint8_t TOT 100

//Voltage for Undervolting and Overvolting
uint16_t VUV = 2
uint16_t VOV = 5

//Teensy 3.1 & 3.2 has got 2048 bytes available
#define FLAG_ADDR 0
#define CORRECT_FLAG 0xFF

#define DCP_MODE_ADDR 1
#define SLAVE_NUM_ADDR 2
#define MAX_MEASURE_CYCLE_DURATION_MS_ADDR 3
#define CELL_IGNORE_INDEX_START_ADDR 4
#define CELL_IGNORE_INDEX_END_ADDR 5
#define GPIO_IGNORE_INDEX_START_ADDR 6
#define GPIO_IGNORE_INDEX_END_ADDR 7
#define TUT_ADDR 8
#define TOT_ADDR 9
//Note VUV & VOV are uint16 | 2 bytes each
#define VUV_ADDR 10
#define VOV_ADDR 12
//uint8_t slave_cfg[6] | 6 bytes
#define SLAVE_CONFIG_ADDR 14

//Loads config from EEPROM if persist flag is correct
void config_load();

//Persists current config to EEPROM and sets persist flag 
void config_persist();

#endif //CONFIG_H