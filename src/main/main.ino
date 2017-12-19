#include <stdint.h>
#include <Arduino.h>
#include "config.h"
#include <SPI.h>
#include "LT_SPI.h"
#include "LTC68042.h"
#include "framework.h"

int8_t pec = 0;

void shut_car_down();

float volts_to_celsius(float cell, float vref);

//This is the entry point. loop() is called after
void setup() {
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE);

  #if DEBUG
    Serial.begin(9600);
    delay(2000);
    Serial.println("> Setup initiating...");
  #endif

  #if DEBUG
    Serial.println("Initializing LTC6804 communication via SPI");
  #endif
  //Initialize communication with attached LTC via SPI
  LTC6804_initialize();
  
  #if DEBUG
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
      check &= (b == slave_cfg[i]);
    }

    if(check == 0){{
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

  if(pec == -1){{
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

  uint16_t cell_codes[SLAVE_NUM][12];
  uint16_t aux_codes[SLAVE_NUM][6];  

  LTC6804_adcv();
  pec = LTC6804_rdcv(CELL_CH_ALL, SLAVE_NUM, cell_codes);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to first demo measurement (cells)!");
    #endif
    shut_car_down();
  }

  LTC6804_adax();

  pec = LTC6804_rdaux(AUX_CH_ALL, SLAVE_NUM, aux_codes);

  if(pec == -1){
    #if DEBUG_PEC
      Serial.println("Slaves sent back incorrect response to first demo measurement (aux/temp)!");
    #endif
    shut_car_down();
  }

  #if DEBUG
    Serial.println("> Setup Complete!");
  #endif
}

//Runs repeatedly after setup()
void loop() {
  #if DEBUG
    Serial.println("Running loop()");
  #endif

  uint32_t measure_cycle_start = 0, measure_cycle_end = 0xFFFFFFFF, measure_cycle_mean = 0;

  uint16_t cell_codes[SLAVE_NUM][12];
  uint16_t aux_codes[SLAVE_NUM][6];

  //Setup has been completed, so we are ready to start making some measurements
  while(1){
    #if DEBUG
      sleep(MEASURE_CYCLE_DEBUG_DELAY_MS);
    #endif

    if(measure_cycle_start == 0 && measure_cycle_end == 0xFFFFFFFF){
      //Ignore this one, because it's the first time this loop runs
    }else{
      uint32_t measure_cycle_duration = measure_cycle_end - measure_cycle_start;

      if(measure_cycle_mean == 0){
        measure_cycle_mean = measure_cycle_duration;
      }else{
        measure_cycle_mean = (measure_cycle_mean + measure_cycle_duration) / 2;
      }

      #if DEBUG
        Serial.print("Measure cycle mean: ");
        Serial.print(measure_cycle_mean);
        Serial.println(" ms");
      #endif
      
      #if DEBUG
        Serial.print("Measure cycle duration: ");
        Serial.print(measure_cycle_duration);
        Serial.println(" ms");
      #endif

      if(measure_cycle_duration > MAX_MEASURE_CYCLE_DURATION_MS){
        #if DEBUG
          Serial.print("> Measure cycle duration > ");
          Serial.print(MAX_MEASURE_CYCLE_DURATION_MS);
          Serial.println(" ms. Shutting car down!");
        #endif
        shut_car_down();
      }
    }
    measure_cycle_start = millis();

    //Transmit Analog-Digital Conversion Start Broadcast to measure CELLS
    //ADCV Command
    LTC6804_adcv();
    delay(3)

    //Start reading everything back
    pec = LTC6804_rdcv(CELL_CH_ALL, SLAVE_NUM, cell_codes);

    if(pec == -1){
      #if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to RDCV!");
      #endif
      shut_car_down();
    }

    for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
      for(uint8_t cell = CELL_IGNORE_INDEX_START; cell < CELL_IGNORE_INDEX_END; cell++){
        #if DEBUG_CELL_VALUES
          Serial.print("Slave #");
          Serial.print(addr);
          Serial.print("'s cell #");
          Serial.print(cell);
          Serial.print(" -> ");
          Serial.print(cell_codes[addr][cell]*0.0001,4);
          Serial.println(" V");
        #endif
      }
    }

    //Transmit Analog-Digital Conversion Start Broadcast to measure GPIOs (Auxiliary)
    //ADAX Command
    LTC6804_adax();
    delay(3);

    //Read GPIO Volts (Temperature values here)
    //RDAUX Command (GPIO Measurements are stored in auxiliary registers)
    pec = LTC6804_rdaux(AUX_CH_ALL, SLAVE_NUM, aux_codes);

    if(pec == -1){
      #if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to RDAUX!");
      #endif
      shut_car_down();
    }

    for(uint8_t addr = 0; addr < SLAVE_NUM; addr++){
      uint16_t vref = aux_codes[addr][5];
      #if DEBUG_CELL_VALUES
        Serial.print("VRef2 -> ");
        Serial.print(vref * 0.0001, 4);
        Serial.println(" V");
      #endif
      for(uint8_t temp = GPIO_IGNORE_INDEX_START; temp < GPIO_IGNORE_INDEX_END; temp++){
        #if DEBUG_CELL_VALUES
          Serial.print("Slave #");
          Serial.print(addr);
          Serial.print("'s GPIO #");
          Serial.print(temp);
          Serial.print(" -> ");
          Serial.print(volts_to_celsius(aux_codes[addr][temp], vref), 4);
          Serial.print(" C <=> ");
          Serial.print(aux_codes[addr][temp] * 0.0001, 4); 
          Serial.println(" V")
        #endif
      }
    }
  
    measure_cycle_end = millis();
  }
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
