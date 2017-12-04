#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <SPI.h>

#define DEBUG 1

//This is the configuration that will be written to every slave
//REFON=1 -> Always awake
//VUV (Undervoltage) & VOV (Overvoltage) Values
const uint8_t slave_cfg[6] = 
{
 0B11111100, //GPIO 1~5 REFON DTEN ADCOPT
 0B00000101, //VUV[7~0]
 0xFF, //VOV[3~0] VOV[11~8]
 0xFF, //VOV[11~4]
 0xFF, //DCC 8~1
 0xFF //DCTO[3~0] DCC 12~9
};

#define SPI_MODE SPI_MODE3
#define SPI_CLOCK_DIV SPI_CLOCK_DIV16

//Cell discharge permitted
#define DCP_MODE DCP_DISABLED

//Number of LTC6811-2 Multicell battery monitors
#define SLAVE_NUM 1

//Voltage for Undervolting and Overvolting
#define VUV 2
#define VOV 5

//Temperature Voltage Read for Undertemping and Overtemping
#define TUT -10
#define TOT 100

#define MAX_MEASURE_CYCLE_DURATION_MS 500

//Cells with index > CELL_IGNORE_INDEX will be ignored from measurements.
//This is useful for cases where your cells on a module 
//are < 12 (max number that the Battery Monitors can read)
//Thus, you should ignore some. In this case we have got
//each monitor observing 10 cells, thus the 11th and 12th 
//are going to be ignored both from the open wire checks and
//the measurements
//Set to -1 in order to measure all
#define CELL_IGNORE_INDEX 10

//Same with CELL_IGNORE_INDEX, but for the GPIOs where the temperature
//sensors (or thermistors) are wired in. The monitors have 5 GPIOs
#define GPIO_IGNORE_INDEX -1

#endif //CONFIG_H