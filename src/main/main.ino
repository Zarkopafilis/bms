#include <stdint.h>
#include <Arduino.h>
#include "framework.h"

#define SERIAL_BAUD_RATE 9600

//Temperature Voltage Read for Undertemping and Overtemping
#define TUT 0
#define TOT 100

//Voltage for Undervolting and Overvolting
#define VUV 2
#define VOV 5

#define SHUTDOWN_PIN 15
#define SHUTDOWN_PIN_IDLE 1

#define CHARGE_PIN 16
#define CHARGE_PIN_IDLE 0

//Cell discharge permitted
//Disabled = 0 | Enabled = 1
#define DCP_MODE DCP_DISABLED

//Number of LTC6811-2 Multicell battery monitors
#define SLAVE_NUM 1

//Index of the battery box, can be anything -- used only to send data metrics to the can bus
//For now 0 => Left and 1 => Right
#define BOX_ID 0

#define MAX_MEASURE_CYCLE_DURATION_MS 500

//Cells with index > CELL_IGNORE_INDEX will be ignored from measurements.
//This is useful for cases where your cells on a module 
//are < 12 (max number that the Battery Monitors can read)
//Thus, you should ignore some. In this case we have got
//each monitor observing 10 cells, thus the 11th and 12th 
//are going to be ignored both from the open wire checks and
//the measurements
//Set to 12(Max Cells) in order to measure all
#define CELL_IGNORE_INDEX_START 0
#define CELL_IGNORE_INDEX_END 12

//Same with CELL_IGNORE_INDEX, but for the GPIOs where the temperature
//Can_Sensors (or thermistors) are wired in. The monitors have 5 GPIOs
//Set to 5(Max GPIOs) in order to measure all
#define GPIO_IGNORE_INDEX_START 0
#define GPIO_IGNORE_INDEX_END 5

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

void shut_car_down();
void critical_callback(BmsCriticalFrame_t);

LT_SPI * lt_spi;
LTC6804_2 * ltc;
BMS * bms;
IVT * ivt;
FlexCAN Can(500000);

#define CAN_SENSOR_NUM 1
Can_Sensor * can_sensors[]{
  ivt
};

inline int charging(){
  return digitalRead(CHARGE_PIN) != CHARGE_PIN_IDLE;
}

//This is the entry point. loop() is called after
void setup() {
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE);

  pinMode(CHARGE_PIN, INPUT);
  #if DEBUG
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);
    Serial.println("Initializing Can0");
  #endif

  Can.begin();

  lt_spi = new LT_SPI();
  ltc = new LTC6804_2(lt_spi);
  ivt = new IVT_Dummy(1);
  bms = new BMS(ltc, ivt, SLAVE_NUM,
          VOV, VUV, TOT, TUT, 
          CELL_IGNORE_INDEX_START, CELL_IGNORE_INDEX_END, GPIO_IGNORE_INDEX_START, GPIO_IGNORE_INDEX_END,
          drive_config,
          &critical_callback,
          &uint16_volts_to_float,
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
    //Gather input from CAN -- there is a need to centralize this because you can't have multiple isntances reading all
    //messages and only grabbing their own, if you read, you consume forever.
    while(Can.available() == 1){
      CAN_message_t msg;
      Can.read(msg);

      for(int i = 0; i < CAN_SENSOR_NUM; i++){
        Can_Sensor * s = can_sensors[i];
        if(msg.id == s->get_id()){
          s->update(msg);
        }
      }
    }

    //Tick BMS
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
  #if DEBUG
    Serial.println(">>> Critical BMS Frame");
  #endif
  return;
  shut_car_down();
}

void shut_car_down(){
  #if DEBUG
    Serial.println(">>>> SHUTTING CAR DOWN <<<<");
  #endif

  digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE == 1 ? 0 : 1);

  while(1){
    //Loop endlessly in chaos
  }
}

