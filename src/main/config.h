/* Only debug config is contained here. Please avoid putting actual configuration code and just
   model everything in a proper OOP-like manner, passing config parameters in the constructor*/
#ifndef CONFIG_H
#define CONFIG_H

#include <EEPROM.h>

#define DEBUG 1
#define DEBUG_PEC 1
#define DEBUG_CELL_VALUES 1
#define DEBUG_TEMP_VALUES 1
#define DEBUG_CURRENT_VALUES 1
#define DEBUG_CAN 1

//Only takes effect when DEBUG = 1
#define MEASURE_CYCLE_DEBUG_DELAY_MS 100

#define CAN_ENABLE 0

#define CONFIG_ADDRESS_VALIDITY 0
#define CONFIG_ADDRESS_VALIDITY_VAL 0xAB

#define CONFIG_ADDRESS_START 10

/* The following are uin16_t's with 2 decimal points of resolution*/
#define CONFIG_OFFSET_UV 0 
#define CONFIG_OFFSET_OV 2

#define CONFIG_OFFSET_UT 4 
#define CONFIG_OFFSET_OT 6


class Configuration{
  public:
    Configuration();

    //YAGN Caching
    float get_undertemp();
    float get_overtemp();

    float get_undervolts();
    float get_overvolts();
  private:
    //Temperature Voltage Read for Undertemping and Overtemping
    static constexpr float default_undertemp = 0, default_overtemp = 100;
    static constexpr float default_undervolt = 4.2, default_overvolt = 5.2;

    void write_uint16(uint16_t start_addr, uint16_t val);
    uint16_t read_uint16(uint16_t start_addr);
};

#endif //CONFIG_H
