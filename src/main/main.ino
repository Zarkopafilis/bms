#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include "LT_SPI.h"
#include "LTC68042.h"

void shut_car_down(){
  #ifdef DEBUG
    Serial.println(">>>> SHUTTING CAR DOWN <<<<");
  #endif

  while(1){
    //Loop endlessly in chaos
  }
}

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
  //Check if every slave is connected and is able to communicate properly
  //RDCFG (Read Configuration) Command

  
  #ifdef DEBUG
    Serial.println("Broadcasting new configuration to slaves");
  #endif
  //Broadcast new configuration
  //REFON=1 -> Always awake
  //VUV (Undervoltage) & VOV (Overvoltage) Values
  //WRCFG (Write Configuration) Command


  #ifdef DEBUG
    Serial.println("Evaluating wether configuration was indeed changed");
  #endif
  //Read Configuration again to evaluate that previous step took effect
  //RDCFG Command

  #ifdef DEBUG
    Serial.println("Performing Open Wire Check");
  #endif
  //Open Wire Check
  //ADOW Command

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