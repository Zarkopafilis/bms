#include <stdint.h>
#include <Arduino.h>
#include "framework.h"

#define SERIAL_BAUD_RATE 9600

//Index of the battery box, can be anything
//Used only to send data metrics to the can bus as the first byte of the bms messages
#define BOX_LEFT 0
#define BOX_RIGHT 1
#define BOX_ID BOX_LEFT;

/* Values for isCharging mode may be different; these are for drive mode */

#define SHUTDOWN_ERROR_CANID 0x600

/* Can Message Layout on shutdown */
// buf[7] => First bit is for left or right battery box, other 6 for the error code
// buf[6 ~ 3] => Value (mV, mA, oC [-127 ~ 127])
// buf[2 ~ 0] => Index (If any) 

#define ERROR_UNKNOWN_CRITICAL 0
#define ERROR_LTC_LOSS 1 /* Pec is wrong */
#define ERROR_IVT_LOSS 2 /* No new IVT measurements on period (possible sensor loss) */
#define ERROR_VOLTS 3 /* Probably never happens in drive mode */
#define ERROR_AMPS 4 /* Overcurrent */
#define ERROR_TEMP 5 /* Too Cold / Too Hot */
#define ERROR_MAX_MEASURE_DURATION 6 /* > 500mS loop time */

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

void shut_car_down();
void critical_callback(BmsCriticalFrame_t);

float volts_to_celsius(float, float);
float uint16_volts_to_float(uint16_t);

LT_SPI * lt_spi;
LTC6804_2 * ltc;
BMS * bms;
IVT * ivt;
FlexCAN Can(500000);

//Collection of all the Can_Sensors that are going
//to be pinged whenever there is a new message
//matching their id
#define CAN_SENSOR_NUM 1
Can_Sensor * can_sensors[]
{
    ivt
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

    prech = 0;
}

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
    Can.begin();

    /* Initialize all the sensors and external hardware as needed.
       They are modelled properly as classes in framework.h*/
    lt_spi = new LT_SPI();
    ltc = new LTC6804_2(lt_spi);
    ivt = new IVT_Dummy(2, 500);
    
    bms = new BMS(ltc, ivt, SLAVE_NUM,
                  VOV, VUV, TOT, TUT,
                  CELL_IGNORE_INDEX_START, CELL_IGNORE_INDEX_END, GPIO_IGNORE_INDEX_START, GPIO_IGNORE_INDEX_END,
                  drive_config,
                  &critical_callback,
                  &uint16_volts_to_float,
                  &volts_to_celsius);

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

        //Tick BMS
        bms->tick();

        Can.write(Liion_Bms_Can_Adapter::VoltageMinMax(bms));

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
            shut_car_down();
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
                if(frame.volts > 0){
#if DEBUG
                Serial.println(frame.volts);
                Serial.println(" V");
#endif
                }else if(frame.temp > 0){
#if DEBUG
                Serial.println(frame.temp);
                Serial.println(" C");
#endif                
                }else if(frame.amps > 0){
#if DEBUG
                Serial.println(frame.amps);
                Serial.println(" A");
#endif
                }
                break;
            case bms_pec_error.mode:
#if DEBUG
                Serial.println("Possible LTC disconnect or malfunction (check for open wires, broken board, liquid damage, etc)");
#endif      
                break;
            case bms_current_error.mode:
#if DEBUG
                Serial.println("Possible IVT Sensor malfunction (can't read data or data invalid/cached for too long)");
#endif      
                break;
            case bms_critical_error.mode: /* Worse case scenario, where we don't know what happened exactly */
#if DEBUG
                Serial.println("Unknown critical error (generalized)!");
#endif      
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
    shut_car_down();
}

/* Actual car shut down code */
// The periodic can message contains a uint32_t in the first 4 byte parts of the array
// That's the error code being transmitted. Look at the top of the file to check what's what
void shut_car_down()
{
#if DEBUG
    Serial.println(">>>> SHUTTING CAR DOWN <<<<");
#endif

    digitalWrite(SHUTDOWN_PIN, SHUTDOWN_PIN_IDLE == 1 ? 0 : 1);
    
    unsigned long lastRefreshTime = millis();

    while(1)
    {   
//        unsigned long nowRefreshTime = millis();
//        if(nowRefreshTime - lastRefreshTime >= 1000){
//          lastRefreshTime = nowRefreshTime;
//          Can.write(periodic);
//        }
        //Loop endlessly in chaos
    }
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
