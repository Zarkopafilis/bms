#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG 1
#define DEBUG_PEC 1
#define DEBUG_CELL_VALUES 1
#define DEBUG_TEMP_VALUES 1

//Only takes effect when DEBUG = 1
#define MEASURE_CYCLE_DEBUG_DELAY_MS 100

//Cells with index > CELL_IGNORE_INDEX will be ignored from measurements.
//This is useful for cases where your cells on a module 
//are < 12 (max number that the Battery Monitors can read)
//Thus, you should ignore some. In this case we have got
//each monitor observing 10 cells, thus the 11th and 12th 
//are going to be ignored both from the open wire checks and
//the measurements
//Set to 12(Max Cells) in order to measure all
#define CELL_IGNORE_INDEX_START 0
#define CELL_IGNORE_INDEX_END 12

//Same with CELL_IGNORE_INDEX, but for the GPIOs where the temperature
//sensors (or thermistors) are wired in. The monitors have 5 GPIOs
//Set to 5(Max GPIOs) in order to measure all
#define GPIO_IGNORE_INDEX_START 0
#define GPIO_IGNORE_INDEX_END 5

#endif //CONFIG_H
