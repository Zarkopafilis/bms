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
const uint16_t MEASURE_CYCLE_DEBUG_DELAY_MS =100;

const uint16_t SERIAL_PORT =9600;


//This is the configuration that will be written to every slave
//REFON=1 -> Always awake
//VUV (Undervoltage) & VOV (Overvoltage) Values
const uint8_t slave_drive_mode_cfg[6] = 
{
 0B00000100, //GPIO 5~1 REFON DTEN ADCOPT
 0B00000000, //VUV[7~0]
 0B00000000, //VOV[3~0] VOV[11~8]
 0B00000000, //VOV[11~4]
 0B00000000, //DCC 8~1
 0B00000000 //DCTO[3~0] DCC 12~9
};

//Cell discharge permitted
//Disabled = 0 | Enabled = 1
#define DCP_MODE DCP_DISABLED

//Number of LTC6811-2 Multicell battery monitors
const uint8_t SLAVE_NUM =1;

const uint16_t MAX_MEASURE_CYCLE_DURATION_MS= 500;

//Cells with index > CELL_IGNORE_INDEX will be ignored from measurements.
//This is useful for cases where your cells on a module 
//are < 12 (max number that the Battery Monitors can read)
//Thus, you should ignore some. In this case we have got
//each monitor observing 10 cells, thus the 11th and 12th 
//are going to be ignored both from the open wire checks and
//the measurements
//Set to 12(Max Cells) in order to measure all
const uint8_t CELL_IGNORE_INDEX_START= 0;
const uint8_t CELL_IGNORE_INDEX_END =12;

//Same with CELL_IGNORE_INDEX, but for the GPIOs where the temperature
//sensors (or thermistors) are wired in. The monitors have 5 GPIOs
//Set to 5(Max GPIOs) in order to measure all
const uint8_t GPIO_IGNORE_INDEX_START =0;
const uint8_t GPIO_IGNORE_INDEX_END= 5;

//Temperature Voltage Read for Undertemping and Overtemping
const uint16_t TUT= 5;
const uint16_t TOT =100;

//Voltage for Undervolting and Overvolting
const uint16_t VUV =2;
const uint16_t VOV =5;

const uint8_t SHUTDOWN_PIN =15;
const uint8_t SHUTDOWN_PIN_IDLE =1;

const uint8_t CHARGE_PIN =16;
const uint8_t CHARGE_PIN_IDLE =0;

//Writes the whole config to serial if DEBUG is 1
void config_print();

#endif //CONFIG_H
