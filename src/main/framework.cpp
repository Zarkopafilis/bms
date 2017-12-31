/* Extensions and additions on LT_SPI and more 
importantly LTC60842 functions to make them work for teensy. */

#include <Arduino.h>
#include "framework.h"
#include "config.h"

float volts_to_celsius(float cell, float vref){
  float R10 = 10;
  float R25 = 10;
  float Aa = 0.003354016;
  float Bb = 0.000256985;
  float Cc = 2.62013 * 0.000001;
  float Dd = 6.38309 * 0.00000001;
  
  float vr = cell * 0.0001;
  
  float r = -(vr * R10) / (vr - vref * 0.0001);
  float T = Aa + Bb * log(r / R25) + Cc * log(r / R25) * log(r / R25) + Dd * log(r / R25) * log(r / R25) * log(r/R25);

  float t = 1/T -272.15;
  return t;
}

float uint16_volts_to_float(uint16_t volts){
  return volts * 0.0001;
}

void output_low(uint8_t pin){
  digitalWrite(pin, LOW);
}

void output_high(uint8_t pin){
  digitalWrite(pin, HIGH);
}

Can_Sensor::Can_Sensor(uint32_t id) : id(id){}

uint32_t Can_Sensor::get_id(){ return this->id; }

IVT::IVT(uint32_t id) : Can_Sensor(id){}

void IVT::update(CAN_message_t message){
      this->val = message.buf[2] << 24 | message.buf[3] << 16 | message.buf[4] << 8 | message.buf[5]; 
}

IVTCurrentMeasureFrame_t IVT::tick(){
  if(old){
    return {IVT_OLD_MEASUREMENT, val};
  }
  old = true;
  return {IVT_SUCCESS, val}; 
}

IVT_Dummy::IVT_Dummy(uint32_t val) : IVT(0xFFFF)
{
  this->val = val;  
}

IVTCurrentMeasureFrame_t IVT_Dummy::tick(){
  return {IVT_SUCCESS, this->val};
}

void IVT_Dummy::update(CAN_message_t message){}

BMS::BMS(LTC6804_2 * ltc, IVT * ivt,
      uint8_t total_ic,
      float overvolts,
      float undervolts,
      float overtemp,
      float undertemp,
      uint8_t cell_start, uint8_t cell_end,
      uint8_t aux_start, uint8_t aux_end,
      const uint8_t conf[6],
      void (* critical_callback)(BmsCriticalFrame_t),
      float (* const uv_to_float)(uint16_t),
      float (* const v_to_celsius)(float, float)) :
        ltc(ltc), ivt(ivt), total_ic(total_ic),
        ov(overvolts), uv(undervolts), ot(overtemp), ut(undertemp),
        cell_start(cell_start), cell_end(cell_end), aux_start(aux_start), aux_end(aux_end),
        config(conf),
        critical_callback(critical_callback), uv_to_float(uv_to_float), v_to_celsius(v_to_celsius)
{
  this->cell_codes = (uint16_t *) malloc(sizeof(uint16_t) * total_ic * (cell_end - cell_start));
  this->aux_codes = (uint16_t *) malloc(sizeof(uint16_t) * total_ic * (aux_end - aux_start));
  
  #if DEBUG
    Serial.println("Writing configuration to slaves");
  #endif

  //Write new configuration to each slave
  //WRCFG (Write Configuration) Command
  uint8_t cfg[total_ic][6];

  for(uint8_t i = 0; i < total_ic; i++){
    for(uint8_t j = 0; j < 6; j++){
        cfg[i][j] = *(config + j);
    }
  }

  ltc->wakeup_sleep();
  ltc->wrcfg(total_ic, cfg);

  #if DEBUG
    Serial.println("Evaluating wether configuration was indeed changed");
  #endif

  uint8_t r_cfg[total_ic][8];
  //Read Configuration again to evaluate that previous step took effect
  //RDCFG Command
  int pec = ltc->rdcfg(total_ic, r_cfg);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.print("Slaves failed to send correct configuration back!");
    #endif
    critical_callback(pec_bms_error);
  }

  for(uint8_t addr = 0; addr < total_ic; addr++){
    uint8_t check = 0xFF;

    //No need to check the 2 PEC bytes again
    for (uint8_t i = 0; i < 6; i++){
      uint8_t b = r_cfg[addr][i];
      check &= (b == *(config + i));
    }

    if(check == 0){
      #if DEBUG
        Serial.print("Slave ");
        Serial.print(addr);
        Serial.println(" was not properly configured!");
      #endif
      critical_callback(critical_bms_error);
    }
  }

  #if DEBUG
    Serial.println("Clearing cell registers of all the slaves");
  #endif 
  //Broadcast cell clear
  //CLRCELL Command
  ltc->clrcell();

  #if DEBUG
    Serial.println("Making sure no cell register bits are stuck");
  #endif
  //Check if every cell is 0xFF in order to determine
  //Wether any possible bits are stuck

  #if DEBUG
    Serial.println("Reading all cell values and comparing with 0xFF");
  #endif
  //Start reading everything
  uint16_t cell_codez[total_ic][12];
  pec = ltc->rdcv(CELL_CH_ALL, total_ic, cell_codez);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to RDCV!");
    #endif
    critical_callback(pec_bms_error);
  }

  for(uint8_t addr = 0; addr < total_ic; addr++){
    for(uint8_t cell = cell_start; cell < cell_end; cell++){
      if(cell_codez[addr][cell] != 0xFFFF){
          #if DEBUG
            Serial.print("Slave #");
            Serial.print(addr);
            Serial.print("'s cell #");
            Serial.print(cell);
            Serial.println(" may have got a bit stuck (not 0xFFFF)!");
          #endif
          critical_callback(critical_bms_error);
      }
    }
  }

  #if DEBUG
    Serial.println("Clearing auxiliary registers of all the slaves");
  #endif
  //Broadcast auxiliary clear
  //CLRAUX Command
  ltc->clraux();

  #if DEBUG
    Serial.println("Making sure no auxiliary register bits are stuck");
  #endif
  //Check if every auxiliary register is 0xFF in order to determine
  //Wether any possible bits are stuck
  uint16_t aux_codez[total_ic][6];
  pec = ltc->rdaux(AUX_CH_ALL, total_ic, aux_codez);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to RDAUX!");
    #endif
    critical_callback(pec_bms_error);
  }

  for(uint8_t addr = 0; addr < total_ic; addr++){
    for(uint8_t aux = 0; aux < 6; aux++){
      if(aux_codez[addr][aux] != 0xFFFF){
        #if DEBUG
          Serial.print("Slave #");
          Serial.print(addr);
          Serial.print("'s aux register #");
          Serial.print(aux);
          Serial.println(" may have got a bit stuck (not 0xFFFF)!");
        #endif
        critical_callback(pec_bms_error);
      }
    }
  }

  #if DEBUG
    Serial.println("Performing 1 measurement and ignoring the results");
  #endif

  ltc->adcv();

  pec = ltc->rdcv(CELL_CH_ALL, total_ic, cell_codez);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to first demo measurement (cells)!");
    #endif
    critical_callback(pec_bms_error);
  }

  ltc->adax();

  pec = ltc->rdaux(AUX_CH_ALL, total_ic, aux_codez);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to first demo measurement (aux/temp)!");
    #endif
    critical_callback(pec_bms_error);
  }

  #if DEBUG
    Serial.println("> Setup Complete...Starting in 2 Seconds!");
  #endif

  //Ensure that we get back correct measurements from the battery monitors on tick
  delay(2000);
}

void BMS::set_cfg(const uint8_t cfg[6]){
  this->config = cfg;
}

BMS::~BMS(){
  free(this->aux_codes);
  free(this->cell_codes);
}

void BMS::tick(){
    uint16_t cell_codez[total_ic][12];
    uint16_t aux_codez[total_ic][6];

    //Write configuration to each slave
    uint8_t cfg[total_ic][6];

    for(uint8_t i = 0; i < total_ic; i++){
      for(uint8_t j = 0; j < 6; j++){
          cfg[i][j] = *(config + j);
      }
    }

    //Write the configuration on every slave on each loop because it gets lost after some time
    ltc->wakeup_sleep();
    ltc->wrcfg(total_ic, cfg);

    //Transmit Analog-Digital Conversion Start Broadcast to measure CELLS
    //ADCV Command
    ltc->adcv();

    //Start reading everything back
    int pec = ltc->rdcv(CELL_CH_ALL, total_ic, cell_codez);

    if(pec == -1){
      #if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to RDCV!");
      #endif
      critical_callback(critical_bms_error);
    }

    for(uint8_t addr = 0; addr < total_ic; addr++){
      for(uint8_t cell = cell_start; cell < cell_end; cell++){
        #if DEBUG_CELL_VALUES
          Serial.print("Slave #");
          Serial.print(addr);
          Serial.print("'s cell #");
          Serial.print(cell);
          Serial.print(" -> ");
          Serial.print(uv_to_float(cell_codez[addr][cell]),4);
          Serial.println(" V");
        #endif
      }
    }

    //Transmit Analog-Digital Conversion Start Broadcast to measure GPIOs (Auxiliary)
    //ADAX Command
    ltc->adax();

    //Read GPIO Volts (Temperature values here)
    //RDAUX Command (GPIO Measurements are stored in auxiliary registers)
    pec = ltc->rdaux(AUX_CH_ALL, total_ic, aux_codez);

    if(pec == -1){
      #if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to RDAUX!");
      #endif
      critical_callback(critical_bms_error);
    }

    for(uint8_t addr = 0; addr < total_ic; addr++){
      uint16_t vref = aux_codez[addr][5];
      #if DEBUG_CELL_VALUES
        Serial.print("VRef2 -> ");
        Serial.print(uv_to_float(vref), 4);
        Serial.println(" V");
      #endif
      for(uint8_t temp = aux_start; temp < aux_end; temp++){
        #if DEBUG_CELL_VALUES
          Serial.print("Slave #");
          Serial.print(addr);
          Serial.print("'s GPIO #");
          Serial.print(temp);
          Serial.print(" -> ");
          Serial.print(v_to_celsius(uv_to_float(aux_codez[addr][temp]), uv_to_float(vref)), 4);
          Serial.print(" C <=> ");
          Serial.print(uv_to_float(aux_codez[addr][temp]), 4); 
          Serial.println(" V");
        #endif
      }
    }

    //After Volts and Temps, read stuff from the current Can_Sensor
    IVTCurrentMeasureFrame_t current_frame = this->ivt->tick();

    if(current_frame.success == 1){
      #if DEBUG_CURRENT_VALUES
        Serial.print("IVT Current: ");
        Serial.print(current_frame.amps);
        Serial.println(" A");
      #endif
      
    }else{
      #if DEBUG_CURRENT_VALUES
        Serial.println("Coudln't read values from IVT Current Can_Sensor!");
      #endif
      critical_callback(current_bms_error);
    }

    for(uint8_t addr = 0; addr < total_ic; addr++){
      for(uint8_t cell = 0; cell < (cell_end - cell_start); cell++){
        *(cell_codes + addr * (cell_end - cell_start) + cell) = cell_codez[addr][cell + cell_start];
      }
    }
    for(uint8_t addr = 0; addr < total_ic; addr++){
      for(uint8_t temp = 0; temp < (aux_end - aux_start); temp++){
        *(this->aux_codes + addr * (aux_end - aux_start) + temp) = aux_codez[addr][temp + aux_start];
      }
    }
}
