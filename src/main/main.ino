#include <stdint.h>
#include <Arduino.h>
#include "framework.h"

#define SERIAL_BAUD_RATE 9600

/* Values for isCharging mode may be different; these are for drive mode */

//Names of some constants are like this in order to be the same
//with the LTC6804 datasheet
//Temperature Voltage Read for Undertemping and Overtemping
#define TUT 0
#define TOT 100

//Voltage for Undervolting and Overvolting
#define VUV 2
#define VOV 5

#define SHUTDOWN_PIN 15
#define SHUTDOWN_PIN_IDLE 1 //If SHUTDOWN_PIN shifts to non SHUTDOWN_PIN_IDLE => Car is latched to shut down

#define CHARGE_PIN 16
#define CHARGE_PIN_IDLE 0 //If CHARGE_PIN == CHARGE_PIN_IDLE => Drive Mode

//Cell discharge permitted
//Disabled = 0 | Enabled = 1
#define DCP_MODE DCP_DISABLED

//Number of LTC6811-2 Multicell battery monitors
//When setting up, make sure that id's start from 0 and go up by increments of 1
#define SLAVE_NUM 1

//Any duration bigger than this is going to shut the car down
#define MAX_MEASURE_CYCLE_DURATION_MS 500

//Only cells in range of START & END are going to be measured.
//If you want to only measure a single cell, just set START to
//the one you want to read and END to START + 1
//This is useful for cases where your cells on a module
//are < 12 (max number that the Battery Monitors can read)
//Thus, you may want to ignore some.
#define CELL_IGNORE_INDEX_START 0
#define CELL_IGNORE_INDEX_END 12

//Same with CELL_IGNORE_INDEX, but for the GPIOs where the temperature
//10k thermistors are wired in. The monitors have 5 GPIOs
#define GPIO_IGNORE_INDEX_START 0
#define GPIO_IGNORE_INDEX_END 5

//This is the configuration that will be written to every slave while driving
//REFON=1 -> Always at idle mode, no sleep
const uint8_t drive_config[6] =
{
    0B00000100, //GPIO 5~1 REFON DTEN ADCOPT
    0B00000000, //VUV[7~0]
    0B00000000, //VOV[3~0] VOV[11~8]
    0B00000000, //VOV[11~4]
    0B00000000, //DCC 8~1
    0B00000000 //DCTO[3~0] DCC 12~9
};

//Will change
const uint8_t charge_config[6] =
{
    0B00000100, //GPIO 5~1 REFON DTEN ADCOPT
    0B00000000, //VUV[7~0]
    0B00000000, //VOV[3~0] VOV[11~8]
    0B00000000, //VOV[11~4]
    0B00000000, //DCC 8~1
    0B00000000 //DCTO[3~0] DCC 12~9
};

void shut_car_down(CAN_message_t);
void critical_callback(BmsCriticalFrame_t);

void tick_can_sensors();

float volts_to_celsius(float, float);
float uint16_volts_to_float(uint16_t);

LT_SPI * lt_spi;
LTC6804_2 * ltc;

BMS * bms;

IVT * ivt;

Other_Battery_Box * other_box;

FlexCAN Can(500000);

Charger * charger;

Configurator * configurator;

//Collection of all the Can_Sensors that are going
//to be pinged whenever there is a new message
//matching their id
#define CAN_SENSOR_NUM 3
Can_Sensor * can_sensors[]
{
    ivt,
    other_box,
    configurator
};

inline int isCharging()
{
    return digitalRead(CHARGE_PIN) != CHARGE_PIN_IDLE;
}

static int prech = 0;

inline int isPrecharging(){
    return prech;
}

void precharge(){
    prech = 1;

    #if BOX_ID == BOX_RIGHT
      while(other_box->get_volts() == 0){
        tick_can_sensors();
      }

      float target_voltage = 0.9 * other_box->get_volts();

      while(ivt->tick().volts < target_voltage){
        tick_can_sensors();
      }
      //Activate some pin
    #else
      bms->tick();
      other_box->send_total_voltage(bms->get_total_voltage());
    #endif

    prech = 0;
}

void charge(){}

//This is the entry point. loop() is called after
void setup()
{
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE);

    pinMode(CHARGE_PIN, INPUT);
#if DEBUG
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);
#endif

#if DEBUG_CAN
    Serial.println("Starting FlexCAN");
#endif

#if CAN_ENABLE
    Can.begin();
#endif 

    /* Initialize all the sensors and external hardware as needed.
       They are modelled properly as classes in framework.h*/
    lt_spi = new LT_SPI();
    ltc = new LTC6804_2(lt_spi);
    
#if CAN_ENABLE
    ivt = new IVT();
#else
    ivt = new IVT_Dummy(2, 500);
#endif
    bms = new BMS(ltc, ivt, SLAVE_NUM,
                  VOV, VUV, TOT, TUT,
                  CELL_IGNORE_INDEX_START, CELL_IGNORE_INDEX_END, GPIO_IGNORE_INDEX_START, GPIO_IGNORE_INDEX_END,
                  drive_config,
                  &critical_callback,
                  &uint16_volts_to_float,
                  &volts_to_celsius);

    other_box = new Other_Battery_Box(&Can);

    configurator = new Configurator(&Can);

    if(isCharging()){
      charger = new Charger_Dummy();
      //charger = new Charger(&Can, 0, 0);
      charge();
      while(1){}
    }

    precharge();
}

//Runs repeatedly after setup()
void loop()
{
#if DEBUG
    Serial.println("Running loop()");
#endif

    uint32_t measure_cycle_start = 0, measure_cycle_end = 0xFFFFFFFF, measure_cycle_mean = 0;

    //Setup has been completed, so we are ready to start making some measurements
    while(1)
    {
#if DEBUG
        delay(MEASURE_CYCLE_DEBUG_DELAY_MS);
#endif

        measure_cycle_start = millis();

        tick_can_sensors();

        //Tick BMS
        bms->tick();

#if CAN_ENABLE
        Can.write(Liion_Bms_Can_Adapter::VoltageMinMax(bms));
#endif 

        measure_cycle_end = millis();

        uint32_t measure_cycle_duration = measure_cycle_end - measure_cycle_start;

        if(measure_cycle_mean == 0)
        {
            measure_cycle_mean = measure_cycle_duration;
        }
        else
        {
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

        if(measure_cycle_duration > MAX_MEASURE_CYCLE_DURATION_MS)
        {
#if DEBUG
            Serial.print("> Measure cycle duration > ");
            Serial.print(MAX_MEASURE_CYCLE_DURATION_MS);
            Serial.println(" ms. Shutting car down!");
#endif
            shut_car_down(Shutdown_Message_Factory::simple(ERROR_MAX_MEASURE_DURATION));
        }
    }
}

/* Handles critical bms frames -- doesn't need to shut the car down--
   for example a measurement from a sensor that did not get updated for
   a BMS tick but that's going to get updated on < MAX_MEASURE_CYCLE_MS
   can be a critical frame (because it's an old measurement), but that
   doesn't make it critical to shut the car down.
   
   This is where the actual logic happens, to shut down the car if something goes wrong
   to properly charging the batteries and precharge.*/
void critical_callback(BmsCriticalFrame_t frame)
{
#if DEBUG
    Serial.print(">>> Critical BMS Frame : ");
#endif

    if(isPrecharging() == 1){

    }else if(isCharging() == 0){ /* Drive mode */
        switch(frame.mode){
            case 0: /* Volts, Temps or Amps*/
                if(frame.volts.value > 0){
#if DEBUG
                  Serial.println(frame.volts.value);
                  Serial.println(" V");
#endif

                  uint32_t mv = frame.volts.value * 1000;
                  shut_car_down(Shutdown_Message_Factory::full(ERROR_VOLTS, mv, frame.volts.index));
                }else if(frame.temp.value > 0){
#if DEBUG
                  Serial.println(frame.temp.value);
                  Serial.println(" C");
#endif                
                  
                  uint32_t celsius = frame.temp.value;
                  shut_car_down(Shutdown_Message_Factory::full(ERROR_TEMP, celsius, frame.temp.index));
                }else if(frame.amps.value > 0){
#if DEBUG
                  Serial.println(frame.amps.value);
                  Serial.println(" A");
#endif
  
                  uint32_t ma = frame.amps.value * 1000;
                  shut_car_down(Shutdown_Message_Factory::full(ERROR_TEMP, ma, frame.amps.index));
                }
                break;
            case bms_pec_error.mode:
#if DEBUG
                Serial.println("Possible LTC disconnect or malfunction (check for open wires, broken board, liquid damage, etc)");
#endif      
                shut_car_down(Shutdown_Message_Factory::simple(ERROR_LTC_LOSS));
                break;
            case bms_current_error.mode:
#if DEBUG
                Serial.println("Possible IVT Sensor malfunction (can't read data or data invalid/cached for too long)");
#endif      
                shut_car_down(Shutdown_Message_Factory::simple(ERROR_IVT_LOSS));
                break;
            case bms_critical_error.mode: /* Worse case scenario, where we don't know what happened exactly */
#if DEBUG
                Serial.println("Unknown critical error (generalized)!");
#endif      
                shut_car_down(Shutdown_Message_Factory::simple(ERROR_UNKNOWN_CRITICAL));
                break;      
            default: /* Should never end up here */
#if DEBUG
                Serial.println("Unrecognizable critical frame mode!");
#endif
                break;
        }
    }else{ /* Charging Mode */

    }

    return;
    shut_car_down(Shutdown_Message_Factory::simple(ERROR_UNKNOWN_CRITICAL));
}

/* Actual car shut down code */
void shut_car_down(CAN_message_t periodic)
{
#if DEBUG
    Serial.println(">>>> SHUTTING CAR DOWN <<<<");
#endif

    digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE == 1 ? 0 : 1);
    
    unsigned long lastRefreshTime = millis();

    while(1)
    {   
        unsigned long nowRefreshTime = millis();
        if(nowRefreshTime - lastRefreshTime >= 1000){
          lastRefreshTime = nowRefreshTime;

#if CAN_ENABLED
          Can.write(periodic);
#endif          
        }
        //Loop endlessly in chaos
    }
}

void tick_can_sensors(){
  #if CAN_ENABLE
        //Gather input from CAN -- there is a need to centralize this because you can't have multiple isntances reading all
        //messages and only grabbing their own, if you read a message, you consume it forever.
        while(Can.available() == 1)
        {
            CAN_message_t msg;
            Can.read(msg);
            /* For every sensor declared, update it with the new message, if IDs match
               BMS needs new sensor data, this is why it's done first.*/
            for(uint32_t i = 0; i < CAN_SENSOR_NUM; i++)
            {
                Can_Sensor * s = can_sensors[i];

                for(uint32_t j = 0; j < s->get_id_num(); j++){
                    if(msg.id == *(s->get_ids() + j))
                    {
                        s->update(msg);
                        break;
                    }
                }  
            }
        }
#endif
}

/* Convert volts to celsius on a 10k thermistor, while given a reference voltage */
float volts_to_celsius(float cell, float vref)
{
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

/* Scaling factor of measuremets */
float uint16_volts_to_float(uint16_t volts)
{
    return volts * 0.0001;
}
