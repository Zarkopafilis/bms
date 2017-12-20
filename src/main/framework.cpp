/* Extensions and additions on LT_SPI and more 
importantly LTC60842 functions to make them work for teensy. */

#include <stdint.h>
#include <Arduino.h>
#include "LT_SPI.h"
#include "LTC68042.h"
#include <SPI.h>
#include "config.h"
#include "framework.h"

int pec = 0;

void setup_drive_mode(){
  #if DEBUG
    Serial.println("Writing configuration to slaves");
  #endif

  //Write new configuration to each slave
  //WRCFG (Write Configuration) Command
  uint8_t cfg[SLAVE_NUM][6];

  for(uint8_t i = 0; i < SLAVE_NUM; i++){
    for(uint8_t j = 0; j < 6; j++){
        cfg[i][j] = slave_drive_mode_cfg[j];
    }
  }

  wakeup_sleep();
  LTC6804_wrcfg(SLAVE_NUM, cfg);

  #if DEBUG
    Serial.println("Evaluating wether configuration was indeed changed");
  #endif

  //Read Configuration again to evaluate that previous step took effect
  //RDCFG Command
  pec = LTC6804_rdcfg(SLAVE_NUM, r_cfg);

  if(pec == 0){
    #if DEBUG_PEC
      Serial.println("Slaves sent correct configuration back");
    #endif
  }else{
    #if DEBUG_PEC
      Serial.print("Slaves failed to send correct configuration back!");
    #endif
    shut_car_down();
  }

  for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
    uint8_t check = 0xFF;

    //No need to check the 2 PEC bytes again
    for (uint8_t i = 0; i < 6; i++){
      uint8_t b = r_cfg[addr][i];
      check &= (b == slave_drive_mode_cfg[i]);
    }

    if(check == 0){
      #if DEBUG
        Serial.print("Slave ");
        Serial.print(addr);
        Serial.println(" was not properly configured!");
      #endif
      shut_car_down();
    }
  }

  #if DEBUG
    Serial.println("Clearing cell registers of all the slaves");
  #endif 
  //Broadcast cell clear
  //CLRCELL Command
  LTC6804_clrcell();

  #if DEBUG
    Serial.println("Making sure no cell register bits are stuck");
  #endif
  //Check if every cell is 0xFF in order to determine
  //Wether any possible bits are stuck

  #if DEBUG
    Serial.println("Reading all cell values and comparing with 0xFF");
  #endif
  //Start reading everything
  uint16_t cell_codes[SLAVE_NUM][12];
  pec = LTC6804_rdcv(CELL_CH_ALL, SLAVE_NUM, cell_codes);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to RDCV!");
    #endif
    shut_car_down();
  }

  for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
    for(uint8_t cell = CELL_IGNORE_INDEX_START; cell < CELL_IGNORE_INDEX_END; cell++){
      if(cell_codes[addr][cell] != 0xFFFF){
          #if DEBUG
            Serial.print("Slave #");
            Serial.print(addr);
            Serial.print("'s cell #");
            Serial.print(cell);
            Serial.println(" may have got a bit stuck (not 0xFFFF)!");
          #endif
          shut_car_down();
      }
    }
  }

  #if DEBUG
    Serial.println("Clearing auxiliary registers of all the slaves");
  #endif
  //Broadcast auxiliary clear
  //CLRAUX Command
  LTC6804_clraux();

  #if DEBUG
    Serial.println("Making sure no auxiliary register bits are stuck");
  #endif
  //Check if every auxiliary register is 0xFF in order to determine
  //Wether any possible bits are stuck
  uint16_t aux_codes[SLAVE_NUM][6];
  pec = LTC6804_rdaux(AUX_CH_ALL, SLAVE_NUM, aux_codes);
  

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to RDAUX!");
    #endif
    shut_car_down();
  }

  for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
    for(uint8_t aux = 0; aux < 6; aux++){
      if(aux_codes[addr][aux] != 0xFFFF){
        #if DEBUG
          Serial.print("Slave #");
          Serial.print(addr);
          Serial.print("'s aux register #");
          Serial.print(aux);
          Serial.println(" may have got a bit stuck (not 0xFFFF)!");
        #endif
        shut_car_down();
      }
    }
  }

  #if DEBUG
    Serial.println("Performing 1 measurement and ignoring the results");
  #endif

  uint16_t cell_codez[SLAVE_NUM][12];
  uint16_t aux_codez[SLAVE_NUM][6];  

  LTC6804_adcv();

  pec = LTC6804_rdcv(CELL_CH_ALL, SLAVE_NUM, cell_codez);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to first demo measurement (cells)!");
    #endif
    shut_car_down();
  }

  LTC6804_adax();

  pec = LTC6804_rdaux(AUX_CH_ALL, SLAVE_NUM, aux_codez);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to first demo measurement (aux/temp)!");
    #endif
    shut_car_down();
  }

  #if DEBUG
    Serial.println("> Setup Complete...Entering Ready to Drive Mode in 2 Seconds!");
  #endif

  //Ensure that we get back correct measurements from the battery monitors
  delay(2000);
}

float volts_to_celsius(float cell, float vref){
  float R10 = 10;
  float R25 = 10;
  float Aa = 0.003354016;
  float Bb = 0.000256985;
  float Cc = 2.62013*0.000001;
  float Dd = 6.38309*0.00000001;
  
  float vr = cell * 0.0001;
  
  float r = -(vr * R10) / (vr - vref * 0.0001);
  float T = Aa + Bb * log(r / R25) + Cc * log(r / R25) * log(r / R25) + Dd * log(r / R25) * log(r / R25) * log(r/R25);

  float t = 1/T -272.15;
  return t;
}

void shut_car_down(){
  return;
  #if DEBUG
    Serial.println(">>>> SHUTTING CAR DOWN <<<<");
  #endif

  digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE == 1 ? 0 : 1);

  while(1){
    //Loop endlessly in chaos
  }
}