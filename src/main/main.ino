#include <stdint.h>
#include <Arduino.h>
#include "config.h"
#include <SPI.h>
#include "LT_SPI.h"
#include "LTC68042.h"
#include "framework.h"

//This is the entry point. loop() is called after
void setup() {
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE);

  pinMode(CHARGE_PIN, INPUT);
  int charging = digitalRead(CHARGE_PIN) != CHARGE_PIN_IDLE;

  #if DEBUG
    Serial.begin(9600);
    delay(2000);
    Serial.println("Initializing LTC6804 communication via SPI");
  #endif

  //Initialize communication with attached LTC via SPI
  LTC6804_initialize();
  
  if(charging != 1){
    setup_drive_mode();
  }
}

//Runs repeatedly after setup()
void loop() {
  #if DEBUG
    Serial.println("Running loop()");
  #endif

  uint32_t measure_cycle_start = 0, measure_cycle_end = 0xFFFFFFFF, measure_cycle_mean = 0;

  uint16_t cell_codes[SLAVE_NUM][12];
  uint16_t aux_codes[SLAVE_NUM][6];

  //Write configuration to each slave
  uint8_t cfg[SLAVE_NUM][6];

  for(uint8_t i = 0; i < SLAVE_NUM; i++){
    for(uint8_t j = 0; j < 6; j++){
        cfg[i][j] = slave_drive_mode_cfg[j];
    }
  }

  //Setup has been completed, so we are ready to start making some measurements
  while(1){
    #if DEBUG
      delay(MEASURE_CYCLE_DEBUG_DELAY_MS);
    #endif

    measure_cycle_start = millis();

    //Write the configuration on every slave on each loop because it gets lost after some time
    wakeup_sleep();
    LTC6804_wrcfg(SLAVE_NUM, cfg);

    //Transmit Analog-Digital Conversion Start Broadcast to measure CELLS
    //ADCV Command
    LTC6804_adcv();

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
}