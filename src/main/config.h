#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG 1

//Number of LTC6811-2 Multicell battery monitors
#define SLAVE_NUM 1

//Voltage for Undervolting and Overvolting
#define VUV 2
#define VOV 5

//Temperature (in Celsius) for Undertemping and Overtemping
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