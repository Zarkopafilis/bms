#include "config.h"
#include <EEPROM.h>

void config_load(){
    byte flag = EEPROM.read(FLAG_ADDR);
    if(flag == CORRECT_FLAG){
        //read configuration
    }
}

void config_persist(){
    EEPROM.write(DCP_MODE_ADDR,DCP_MODE);
    EEPROM.write(SLAVE_NUM_ADDR,SLAVE_NUM);
    EEPROM.write(MAX_MEASURE_CYCLE_DURATION_MS_ADDR,MAX_MEASURE_CYCLE_DURATION_MS);
    EEPROM.write(CELL_IGNORE_INDEX_START_ADDR,CELL_IGNORE_INDEX_START);
    EEPROM.write(CELL_IGNORE_INDEX_END_ADDR,CELL_IGNORE_INDEX_END);
    EEPROM.write(GPIO_IGNORE_INDEX_START_ADDR,GPIO_IGNORE_INDEX_START);
    EEPROM.write(GPIO_IGNORE_INDEX_END_ADDR,GPIO_IGNORE_INDEX_END);
    EEPROM.write(TUT_ADDR,TUT);
    EEPROM.write(TOT_ADDR,TOT);

    EEPROM.write(VUV_ADDR, VUV && 0xFF);
    EEPROM.write(VUV_ADDR + 1, VUV >> 8);

    EEPROM.write(VOV_ADDR, VOV && 0xFF);
    EEPROM.write(VOV_ADDR + 1, VOV >> 8);

    for(uint8_t i = 0; i < 6; i++){
        EEPROM.write(SLAVE_CONFIG_ADDR + i, slave_cfg[i]);
    }

    EEPROM.write(FLAG_ADDR, CORRECT_FLAG);
}