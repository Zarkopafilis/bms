/* Only debug config is contained here. Please avoid putting actual configuration code and just
   model everything in a proper OOP-like manner, passing config parameters in the constructor*/
#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG 1
#define DEBUG_PEC 1
#define DEBUG_CELL_VALUES 1
#define DEBUG_TEMP_VALUES 1
#define DEBUG_CURRENT_VALUES 1
#define DEBUG_CAN 1

//Only takes effect when DEBUG = 1
#define MEASURE_CYCLE_DEBUG_DELAY_MS 100

#define CAN_ENABLE 0

#endif //CONFIG_H
