/* Extensions and additions on LT_SPI and more
importantly LTC60842 functions to make them work for teensy. */

#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <stdint.h>
#include "LTC6804_2.h"
#include "FlexCAN.h"
#include "config.h"

#define DRIVE_MODE 0
#define CHARGE_MODE 1

void output_low(uint8_t pin);
void output_high(uint8_t pin);

//Index of the battery box, can be anything
//Used only to send data metrics to the can bus as the first byte of the bms messages
#define BOX_LEFT 0
#define BOX_RIGHT 1
#define BOX_ID BOX_LEFT

// Look at http://liionbms.com/php/standards.php
#if BOX_ID == BOX_LEFT
  #define LIION_START_CANID 0x64D
#else
  #define LIION_START_CANID 0x61D
#endif


#define LIION_VOLT_MIN_MAX_OFFSET 3

#define CHARGER_COMMAND_CANID 0x618

#define SEND_ALL_VOLTS_REQUEST_CANID 0x6FE
#define SEND_ALL_VOLTS_RESPONSE_CANID 0x6FF


#ifndef IVT_CURRENT_CANID
  #define IVT_CURRENT_CANID 0x521
#endif
#ifndef IVT_VOLTAGE_CANID
  #define IVT_VOLTAGE_CANID 0x522
#endif

#if BOX_ID == BOX_LEFT
  #define OTHER_BOX_VOLTAGE_CANID 0x604
  #define CURRENT_BOX_VOLTAGE_CANID 0x605
#else
  #define OTHER_BOX_VOLTAGE_CANID 0x605
  #define CURRENT_BOX_VOLTAGE_CANID 0x604
#endif


/* Every time a configuration message is received, byte(s) are written into the EEPROM:*/
//buf[7] : Start Address
//buf[6] : # Of Bytes to write (1~6)
//buf[5~0] : Actual values
//if address == 0xFF, config is being reset on defaults on next boot.
#define CONFIGURATION_CANID 0x6AA

//first bit of buf[8] is box left (0) and box right (1)
#define CONFIGURATION_ACK_CANID 0x6AB


/* Can Message Layout on shutdown */
// buf[7] => First bit is for left or right battery box, other 6 for the error code
// buf[6 ~ 3] => Value (mV, mA, Celsius (negative ~ positive)])
// buf[2] => Index (If any) 
#define SHUTDOWN_ERROR_CANID 0x600

#if BOX_ID == BOX_LEFT
  #define ERROR_OFFSET 0
#else
  #define ERROR_OFFSET 32 //B 100000
#endif

#define ERROR_UNKNOWN_CRITICAL 0
#define ERROR_LTC_LOSS 1 /* Pec is wrong */
#define ERROR_IVT_LOSS 2 /* No new IVT measurements on period (possible sensor loss) */
#define ERROR_VOLTS 3 /* Probably happens in drive mode */
#define ERROR_AMPS 4 /* Overcurrent */
#define ERROR_TEMP 5 /* Too Cold / Too Hot */
#define ERROR_MAX_MEASURE_DURATION 6 /* > 500mS loop time */

#define IVT_SUCCESS 1
#define IVT_OLD_MEASUREMENT -1

typedef struct ivt_measure_frame
{
    int success;//see constants declared above
    float amps;
    float volts;
} IVTMeasureFrame_t;

//Interface for all Can_Sensors that send data via can
//Should handle message updating and/or caching on the underlying implementation
class Can_Sensor
{
public:
    virtual void update(CAN_message_t message) = 0;

    virtual uint32_t const * get_ids() = 0;
    virtual uint32_t get_id_num() = 0;
};

//Current measure Can_Sensor that returns measure frames
//and caches last successful measurement
class IVT : public Can_Sensor
{
public:
    virtual IVTMeasureFrame_t tick();
    void update(CAN_message_t message);

    uint32_t const * get_ids();
    uint32_t get_id_num();

protected:
    bool old_amps = false;
    bool old_volts = false;
    //Last successful measurement frame
    float amps = 0;
    float volts = 0;

    static const uint32_t id_num = 2;
    const uint32_t ids[id_num] = {IVT_CURRENT_CANID, IVT_VOLTAGE_CANID};
};

//Dummy/Fake IVT sensor that always returns the value provided on
//the constructor, as a successful/new measurement
class IVT_Dummy : public IVT
{
public:
    IVT_Dummy(float amps, float volts);
    IVTMeasureFrame_t tick();
    void update(CAN_message_t message);
    private:
        const float amps;
        const float volts;
};

typedef struct float_index_tuple{
  float value;
  uint8_t index;
} Float_Index_Tuple_t;

static constexpr Float_Index_Tuple_t empty_float_index = {0,0};

//If volts = temp = 0 and mode != 1 | 2 => warning
//If volts = temp = amps = -1 => critical error
//If volts = temp = 0 , amps = -1 => Can_Sensor loss
//Else, normal values indicate whats off-limit
//In general, if mode < 0, something bad happened,
//else, actual critical frame is provided and the non-zero member
//tells what went wrong
typedef struct bms_critical_frame
{
    int mode;
    Float_Index_Tuple_t volts;
    Float_Index_Tuple_t temp;
    Float_Index_Tuple_t amps;
} BmsCriticalFrame_t;

static constexpr BmsCriticalFrame_t bms_critical_error{-10, empty_float_index, empty_float_index, empty_float_index};
static constexpr BmsCriticalFrame_t bms_pec_error{-1, empty_float_index, empty_float_index, empty_float_index};
static constexpr BmsCriticalFrame_t bms_current_error{-2, empty_float_index, empty_float_index, empty_float_index};

//The actual,non-dumb BMS class. It monitors through the Can_Sensors (Currently LTC6804_2 and IVT). You need to plug in
//Some logic for it to work properly. All it does is to report values as a 'Critical BMS Frame'
//The whole system works in a matter of 'ticks' as a dinstinct time frame. Previous values are cached.
class BMS
{
    public:
        BMS(LTC6804_2 * ltc, IVT * ivt,
            uint8_t total_ic,
            float overvolts,
            float undervolts,
            float overtemp,
            float undertemp,
            uint8_t cell_start, uint8_t cell_end,
            uint8_t aux_start, uint8_t aux_end,
            const uint8_t conf[6],
            void (* critical_callback)(BmsCriticalFrame_t),
            float (* uv_to_float)(uint16_t),
            float (* v_to_celsius)(float, float));
    
        ~BMS();
    
        void tick();
        void set_cfg(const uint8_t conf[6]);
    
        uint16_t * cell_codes;
        uint16_t * aux_codes;
    
        LTC6804_2 * const ltc;
        IVT * const ivt;
    
        const uint8_t total_ic;
        const float ov, uv, ot, ut;
        const uint8_t cell_start, cell_end, aux_start, aux_end;

    protected:
      uint8_t const * config;

    public:
    
      void (* const critical_callback)(BmsCriticalFrame_t);
      float (* const uv_to_float)(uint16_t);
      float (* const v_to_celsius)(float, float);
  
      /* The following return the min/max value along with the index of it [slave * (range) + slot] */
      Float_Index_Tuple_t get_volts(bool greater);
      Float_Index_Tuple_t get_temp(bool greater);
  
      Float_Index_Tuple_t get_min_volts();
      Float_Index_Tuple_t get_max_volts();
      
      Float_Index_Tuple_t get_min_temp();
      Float_Index_Tuple_t get_max_temp();

      float get_total_voltage();
};

//Drop in replacement for http://liionbms.com/php/standards.php
//Produces can messages by using the Bms
class Liion_Bms_Can_Adapter{
  public:
    static CAN_message_t VoltageMinMax(BMS * bms);
};

// Accepts configuration 
//and sends out proper can messages to the actual charger
class Charger{
  public:
    Charger(FlexCAN * can, uint16_t initial_volts, uint16_t initial_amps);

    virtual void send_charge_message();

    void set_volts(uint16_t v);
    void set_amps(uint16_t a);
    void set_volts_amps(uint16_t v, uint16_t a);
    
  protected:
    FlexCAN * const can;
    uint16_t volts = 0;
    uint16_t amps = 0;
};

class Charger_Dummy : public Charger{
  public: 
    Charger_Dummy();
  
    void send_charge_message(); 
};

class Shutdown_Message_Factory{
  public:
    static CAN_message_t simple(uint8_t error);

    static CAN_message_t data(uint8_t error, uint32_t data);

    static CAN_message_t full(uint8_t error, uint32_t data, uint8_t index);
};

//The other battery box
class Other_Battery_Box : public Can_Sensor{
    public:
      Other_Battery_Box(FlexCAN * can);
    
      void update(CAN_message_t message);

      void send_total_voltage(float volts);

      float get_volts();
  
      uint32_t const * get_ids();
      uint32_t get_id_num();
    protected:
      static const uint32_t id_num = 1;
      const uint32_t ids[id_num] = {OTHER_BOX_VOLTAGE_CANID};

      FlexCAN * const can; 

      //volts being 0 is illegal value // uncached
      float volts = 0;
};

class Configurator : public Can_Sensor{
  public:
      Configurator(FlexCAN * can);
  
      void update(CAN_message_t message);
  
      uint32_t const * get_ids();
      uint32_t get_id_num();
  protected:
     static const uint32_t id_num = 1;
     const uint32_t ids[id_num] = {CONFIGURATION_CANID};

     FlexCAN * const can; 
};

#endif //FRAMEWORK_H
