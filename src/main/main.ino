#include <stdint.h>
#include <Arduino.h>
#include "framework.h"

#define SERIAL_BAUD_RATE 9600

//Temperature Voltage Read for Undertemping and Overtemping
#define TUT 0
#define TOT 100

//Voltage for Undervolting and Overvolting
#define VUV 2;
#define VOV 5;

#define SHUTDOWN_PIN 15
#define SHUTDOWN_PIN_IDLE 1

#define CHARGE_PIN 16
#define CHARGE_PIN_IDLE 0

//Cell discharge permitted
//Disabled = 0 | Enabled = 1
#define DCP_MODE DCP_DISABLED

//Number of LTC6811-2 Multicell battery monitors
#define SLAVE_NUM =1;

#define MAX_MEASURE_CYCLE_DURATION_MS= 500;

//This is the configuration that will be written to every slave while driving
//REFON=1 -> Always awake
//VUV (Undervoltage) & VOV (Overvoltage) Values
const uint8_t drive_config[6] = 
{
 0B00000100, //GPIO 5~1 REFON DTEN ADCOPT
 0B00000000, //VUV[7~0]
 0B00000000, //VOV[3~0] VOV[11~8]
 0B00000000, //VOV[11~4]
 0B00000000, //DCC 8~1
 0B00000000 //DCTO[3~0] DCC 12~9
};

//This is the configuration that will be written to every slave while charging
//REFON=1 -> Always awake
//VUV (Undervoltage) & VOV (Overvoltage) Values
const uint8_t charge_config[6] = 
{
 0B00000100, //GPIO 5~1 REFON DTEN ADCOPT
 0B00000000, //VUV[7~0]
 0B00000000, //VOV[3~0] VOV[11~8]
 0B00000000, //VOV[11~4]
 0B00000000, //DCC 8~1
 0B00000000 //DCTO[3~0] DCC 12~9
};

void critical_callback(BmsCriticalFrame_t);

LT_SPI * lt_spi;
LTC6804_2 * ltc;
BMS * bms;

//This is the entry point. loop() is called after
void setup() {
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE);

  pinMode(CHARGE_PIN, INPUT);
  int charging = digitalRead(CHARGE_PIN) != CHARGE_PIN_IDLE;

  #if DEBUG
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);
    Serial.println("Initializing LTC6804 communication via SPI");
  #endif

  lt_spi = &LT_SPI();
  ltc = &LTC6804_2(lt_spi);
  bms = &(ltc, SLAVE_NUM, charging == 1 ? CHARGE_MODE : DRIVE_MODE,
          VOV, VUV, TOT, TUT, 
          &critical_callback,
          &uint16_volts_to_float
          &volts_to_celsius);
}

//Runs repeatedly after setup()
void loop() {
  #if DEBUG
    Serial.println("Running loop()");
  #endif

  uint32_t measure_cycle_start = 0, measure_cycle_end = 0xFFFFFFFF, measure_cycle_mean = 0;

  //Setup has been completed, so we are ready to start making some measurements
  while(1){
    #if DEBUG
      delay(MEASURE_CYCLE_DEBUG_DELAY_MS);
    #endif

    measure_cycle_start = millis();

    bms->tick();
  
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

void critical_callback(BmsCriticalFrame_t frame){
  return;
  #if DEBUG
    Serial.println(">>>> SHUTTING CAR DOWN <<<<");
  #endif

  digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE == 1 ? 0 : 1);

  while(1){
    //Loop endlessly in chaos
  }
}
