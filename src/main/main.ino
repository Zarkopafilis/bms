#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include "LT_SPI.h"
#include "LTC68042.h"
#include "framework.h"

//REFON=1 -> Always awake
//VUV (Undervoltage) & VOV (Overvoltage) Values
const uint8_t slave_cfg[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

int8_t pec = 0;

void shut_car_down();

//This is the entry point. loop() is called after
void setup() {
    #ifdef DEBUG
      Serial.begin(9600);
      Serial.println("> Setup initiating...")
    #endif

  #ifdef DEBUG
    Serial.println("Initializing LTC6804 communication via SPI");
  #endif
  //Initialize communication with attached LTC via SPI
  LTC6804_initialize();
  
  #ifdef DEBUG
    Serial.println("Checking if every slave is connected and able to communicate properly");
  #endif

  //Till we write the configuration we need to tell the battery monitor
  //To wake up before each command

  //Check if every slave is connected and is able to communicate properly
  //RDCFG (Read Configuration) Command
  uint8_t r_cfg[SLAVE_NUM][8];

  actual_wakeup_idle();
  pec = LTC6804_rdcfg(SLAVE_NUM, r_cfg);

  if(pec == 0){
    #ifdef DEBUG
      Serial.print("Slave ");
      Serial.print(addr);
      Serial.println(" connected");
    #endif
  }else{
    #ifdef DEBUG
      Serial.print("Slave ");
      Serial.print(addr);
      Serial.println(" transmitted wrong PEC when checking comms!");
    #endif
    shut_car_down();
  }
  
  #ifdef DEBUG
    Serial.println("Writing configuration to slaves");
  #endif

  //Write new configuration to each slave
  //WRCFG (Write Configuration) Command
  uint8_t cfg[SLAVE_NUM][6];

  for(uint8_t i = 0; i < SLAVE_NUM; i++){
    for(uint8_t j = 0; j < 6; j++){
        cfg[i][j] = slave_cfg[j];
    }
  }

  actual_wakeup_idle();
  LTC6804_wrcfg(SLAVE_NUM, cfg);

  #ifdef DEBUG
    Serial.println("Evaluating wether configuration was indeed changed");
  #endif

  //Read Configuration again to evaluate that previous step took effect
  //RDCFG Command
  pec = LTC6804_rdcfg(SLAVE_NUM, r_cfg);

  if(pec == 0){
    #ifdef DEBUG
      Serial.print("Slave ");
      Serial.print(addr);
      Serial.println(" connected");
    #endif
  }else{
    #ifdef DEBUG
      Serial.print("Slave ");
      Serial.print(addr);
      Serial.println(" transmitted wrong PEC when checking if configuration was properly applied");
    #endif
    shut_car_down();
  }

  for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
    uint8_t check = 0xFF;

    //No need to check the 2 PEC bytes again
    for (uint8_t i = 0; i < 6; i++){
      uint8_t byte = r_cfg[addr][i];
      check &= (byte == slave_cfg[i]);
    }

    if(check == 0xFF){
      #ifdef DEBUG
        Serial.print("Slave ");
        Serial.print(addr);
        Serial.println(" got properly configured");
      #endif
    }else{
      #ifdef DEBUG
        Serial.print("Slave ");
        Serial.print(addr);
        Serial.println(" was not properly configured!");
      #endif
      shut_car_down();
    }
  }

  #ifdef DEBUG
    Serial.println("Performing Open Wire Check");
  #endif
  //Open Wire Check
  //ADOW Command
  pec = LTC6804_adow(SLAVE_NUM, CELL_IGNORE_INDEX);

  if(pec == 0){
    #ifdef DEBUG
      Serial.println("All slave wires are properly connected");
    #endif
  }else{
    #ifdef DEBUG
      Serial.println("Some wires are not connected to the Battery Monitor!");
    #endif
    shut_car_down();
  }

  #ifdef DEBUG
    Serial.println("Clearing cell registers of all the slaves");
  #endif 
  //Broadcast cell clear
  //CLRCELL Command
  LTC6804_clrcell();

  #ifdef DEBUG
    Serial.println("Making sure no cell register bits are stuck");
  #endif
  //Check if every cell is 0xFF in order to determine
  //Wether any possible bits are stuck

  #ifdef DEBUG
    Serial.println("Clearing auxiliary registers of all the slaves");
  #endif
  //Broadcast auxiliary clear
  //CLRAUX Command

  #ifdef DEBUG
    Serial.println("Making sure no auxiliary register bits are stuck");
  #endif
  //Check if every auxiliary register is 0xFF in order to determine
  //Wether any possible bits are stuck

  #ifdef DEBUG
    Serial.println("Clearing status registers of all the slaves");
  #endif
  //Broadcast status clear
  //CLRSTAT Command

  #ifdef DEBUG
    Serial.println("Making sure no status register bits are stuck");
  #endif
  //Check if every status register is 0xFF in order to determine
  //Wether any possible bits are stuck

  #ifdef DEBUG
    Serial.println("Slowly reading cell values of everything after 2 ADC passes to make sure battery is ok");
  #endif  

  #ifdef DEBUG
    Serial.println("> Setup Complete!");
  #endif
}

//Runs repeatedly after setup()
void loop() {
  #ifdef DEBUG
    Serial.println("Running loop()");
  #endif

  uint32_t measure_cycle_start = 0, measure_cycle_end = 0xFFFFFFFF;

  //Setup has been completed, so we are ready to start making some measurements
  while(1){
    if(measure_cycle_start == 0 && measure_cycle_end == 0xFFFFFFFF){
      //Ignore this one, because it's the first time this loop runs
    }else{
      uint32_t measure_cycle_duration = measure_cycle_end - measure_cycle_start;
      #ifdef DEBUG
        Serial.print("Measure cycle duration: ");
        Serial.print(measure_cycle_duration);
        Serial.println(" ms")
      #endif

      if(measure_cycle_duration > MAX_MEASURE_CYCLE_DURATION_MS){
          #ifdef DEBUG
            Serial.print("> Measure cycle duration > ");
            Serial.print(MAX_MEASURE_CYCLE_DURATION_MS);
            Serial.println(" ms. Shutting car down!");
          #endif
        shut_car_down();
      }
    }
    measure_cycle_start = millis();

    //Transmit Analog-Digital Conversion Start Broadcast
    //ADCS Command
    
    //Poll each slave 1 by 1

    //Then, start reading results of status registers
    //In order to check undervoltage and overvoltage of each cell


    measure_cycle_end = millis();
  }
}

void shut_car_down(){
  #ifdef DEBUG
    Serial.println(">>>> SHUTTING CAR DOWN <<<<");
  #endif

  while(1){
    //Loop endlessly in chaos
  }
}