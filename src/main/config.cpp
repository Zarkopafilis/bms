#include "config.h"

Configuration::Configuration(){
  if(EEPROM.read(CONFIG_ADDRESS_VALIDITY) != CONFIG_ADDRESS_VALIDITY_VAL || EEPROM.read(CONFIG_ADDRESS_VALIDITY) == 0xFF || EEPROM.read(CONFIG_ADDRESS_VALIDITY) == 0x00){
    uint16_t uv = default_undervolt * 100;
    write_uint16(CONFIG_ADDRESS_START + CONFIG_OFFSET_UV, uv);
    
    uint16_t ov = default_overvolt * 100;
    write_uint16(CONFIG_ADDRESS_START + CONFIG_OFFSET_OV, ov);
    
    uint16_t ut = default_undertemp * 100;
    write_uint16(CONFIG_ADDRESS_START + CONFIG_OFFSET_UT, ut);
    
    uint16_t ot = default_overtemp * 100;
    write_uint16(CONFIG_ADDRESS_START + CONFIG_OFFSET_OT, ot);
    
    EEPROM.write(CONFIG_ADDRESS_VALIDITY, CONFIG_ADDRESS_VALIDITY_VAL);
  }
}

float Configuration::get_undertemp(){ return read_uint16(CONFIG_ADDRESS_START + CONFIG_OFFSET_UT) * 0.01; }
float Configuration::get_overtemp(){ return read_uint16(CONFIG_ADDRESS_START + CONFIG_OFFSET_OT) * 0.01; }

float Configuration::get_undervolts(){ return read_uint16(CONFIG_ADDRESS_START + CONFIG_OFFSET_UV) * 0.01; }
float Configuration::get_overvolts(){ return read_uint16(CONFIG_ADDRESS_START + CONFIG_OFFSET_OV) * 0.01; }

void Configuration::write_uint16(uint16_t start_addr, uint16_t val){
  EEPROM.write(start_addr, (val >> 8) & 0xFF); 
  EEPROM.write(start_addr + 1, (val) & 0xFF); 
}

uint16_t Configuration::read_uint16(uint16_t start_addr){
  return EEPROM.read(start_addr) << 8 | EEPROM.read(start_addr + 1);
}

