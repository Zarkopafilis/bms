#include "config.h"
#include <EEPROM.h>

uint8_t slave_cfg[6] = 
{
 0B00000100, //GPIO 5~1 REFON DTEN ADCOPT
 0B00000000, //VUV[7~0]
 0B00000000, //VOV[3~0] VOV[11~8]
 0B00000000, //VOV[11~4]
 0B00000000, //DCC 8~1
 0B00000000 //DCTO[3~0] DCC 12~9
};

uint8_t DCP_MODE = DCP_DISABLED;

uint8_t SLAVE_NUM = 1;

uint16_t MAX_MEASURE_CYCLE_DURATION_MS = 500;

uint8_t CELL_IGNORE_INDEX_START = 2;
uint8_t CELL_IGNORE_INDEX_END = 12;

uint8_t GPIO_IGNORE_INDEX_START = 0;
uint8_t GPIO_IGNORE_INDEX_END = 5;

uint16_t TUT = 5;
uint16_t TOT = 100;

uint16_t VUV = 2;
uint16_t VOV = 5;

void config_load(){
    byte flag = EEPROM.read(FLAG_ADDR);
    if(flag == CORRECT_FLAG){
        //read configuration
        #if DEBUG
            Serial.println("Configuration flag is set, loading data from EEPROM");
        #endif
        DCP_MODE = EEPROM.read(DCP_MODE_ADDR);
        SLAVE_NUM = EEPROM.read(SLAVE_NUM_ADDR);
        MAX_MEASURE_CYCLE_DURATION_MS = EEPROM.read(MAX_MEASURE_CYCLE_DURATION_MS_ADDR);
        CELL_IGNORE_INDEX_START = EEPROM.read(CELL_IGNORE_INDEX_START_ADDR);
        CELL_IGNORE_INDEX_END = EEPROM.read(CELL_IGNORE_INDEX_END_ADDR);
        GPIO_IGNORE_INDEX_START = EEPROM.read(GPIO_IGNORE_INDEX_START_ADDR);
        GPIO_IGNORE_INDEX_END = EEPROM.read(GPIO_IGNORE_INDEX_END_ADDR);

        uint8_t lsb = EEPROM.read(TUT_ADDR);
        uint8_t msb = EEPROM.read(TUT_ADDR + 1);
        TUT = msb << 8 | lsb;

        lsb = EEPROM.read(TOT_ADDR);
        msb = EEPROM.read(TOT_ADDR + 1);
        TOT = msb << 8 | lsb;

        lsb = EEPROM.read(VUV_ADDR);
        msb = EEPROM.read(VUV_ADDR + 1);
        VUV = msb << 8 | lsb;

        lsb = EEPROM.read(VOV_ADDR);
        msb = EEPROM.read(VOV_ADDR);
        VOV = msb << 8 | lsb;

        for(uint8_t i = 0; i < 6; i++){
            slave_cfg[i] = EEPROM.read(SLAVE_CONFIG_ADDR + i);
        }
    }else{
        #if DEBUG
            Serial.println("Configuration flag is not set, switching to defaults");
        #endif
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
    EEPROM.write(TUT_ADDR, ((uint8_t)(TUT && 0xFF)));
    EEPROM.write(TUT_ADDR + 1,((uint8_t) TUT >> 8));

    EEPROM.write(TOT_ADDR, ((uint8_t)(TOT && 0xFF)));
    EEPROM.write(TOT_ADDR + 1,((uint8_t) TOT >> 8));

    EEPROM.write(VUV_ADDR, ((uint8_t)(VUV && 0xFF)));
    EEPROM.write(VUV_ADDR + 1,((uint8_t) VUV >> 8));

    EEPROM.write(VOV_ADDR, ((uint8_t)(VOV && 0xFF)));
    EEPROM.write(VOV_ADDR + 1, ((uint8_t)VOV >> 8));

    for(uint8_t i = 0; i < 6; i++){
        EEPROM.write(SLAVE_CONFIG_ADDR + i, slave_cfg[i]);
    }

    EEPROM.write(FLAG_ADDR, CORRECT_FLAG);
}

void config_print(){
    #if DEBUG
        Serial.print("DCP_MODE -> ");
        Serial.println(DCP_MODE);
        Serial.print("SLAVE_NUM -> ");
        Serial.println(SLAVE_NUM);
        Serial.print("MAX_MEASURE_CYCLE_DURATION_MS -> ");
        Serial.println(MAX_MEASURE_CYCLE_DURATION_MS);
        Serial.print("CELL_IGNORE_INDEX_START -> ");
        Serial.println(CELL_IGNORE_INDEX_START);
        Serial.print("CELL_IGNORE_INDEX_END -> ");
        Serial.println(CELL_IGNORE_INDEX_END);
        Serial.print("GPIO_IGNORE_INDEX_START -> ");
        Serial.println(GPIO_IGNORE_INDEX_START);
        Serial.print("GPIO_IGNORE_INDEX_END -> ");
        Serial.println(GPIO_IGNORE_INDEX_END_ADDR);
        Serial.print("TUT -> ");
        Serial.println(TUT);
        Serial.print("TOT -> ");
        Serial.println(TOT);
        Serial.print("VUV -> ");
        Serial.println(VUV);
        Serial.print("VOV -> ");
        Serial.println(VOV);

        for(uint8_t i = 0; i < 6; i++){
            Serial.print("slave_cfg[");
            Serial.print(i);
            Serial.print("] -> ");
            Serial.println(slave_cfg[i]);
        }
    #endif
}
