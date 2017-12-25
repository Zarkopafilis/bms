/* Extensions and additions on LT_SPI and more 
importantly LTC60842 functions to make them work for teensy. */

#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <stdint.h>
#include "LTC6804_2.h"
#include "FlexCAN.h"

#define DRIVE_MODE 0
#define CHARGE_MODE 1

void output_low(uint8_t pin);
void output_high(uint8_t pin);

//10K Thermistor on VRef2 and GPIOx Pin
float volts_to_celsius(float cell, float vref);
float uint16_volts_to_float(uint16_t volts);

#ifndef IVT_CURRENT_CANID
#define IVT_CURRENT_CANID 0x521
#endif

typedef struct ivt_amps_frame{
  int success;//1 for success, 0 for failure (no frames waiting)
  uint32_t amps;
} IVTCurrentMeasureFrame_t;

//Dumb current measure sensor that returns measure frames and caches last successful measurement
class IVT {
  public:
    IVT(FlexCAN * can);
    virtual IVTCurrentMeasureFrame_t tick();
    //Last successful measurement frame
    IVTCurrentMeasureFrame_t frame;

  protected:
    FlexCAN * const can;  
};

//This dummy ivt sensor only returns successful measurements of 0 amps
class IVT_Dummy : public IVT{
  public:
    IVT_Dummy();
    IVTCurrentMeasureFrame_t tick();
};

//If volts = temp = 0 and mode != 1 | 2 => warning
//If volts = temp = amps = -1 => critical error
//If volts = temp = 0 , amps = -1 => sensor loss
//Else, normal values indicate whats off-limit
typedef struct bms_critical_frame {
  uint8_t mode;
  float volts;
  float temp;
  uint32_t amps;
} BmsCriticalFrame_t;

const BmsCriticalFrame_t critical_bms_error{0 , -1 , -1, 0xFFFFFFFF};

//The actual,non-dumb BMS class. It monitors through the sensors (Currently LTC6804_2 and IVT). You need to plug in
//Some logic for it to work properly. All it does is to report values as a 'Critical BMS Frame'
//The whole system works in a matter of 'ticks' as a dinstinct time frame. Previous values are cached.
class BMS{
  public:
  BMS(LTC6804_2 * ltc, IVT * ivt,
      uint8_t total_ic,
      uint8_t mode,
      float overvolts,
      float undervolts,
      float overtemp ,
      float undertemp,
      uint8_t cell_start, uint8_t cell_end,
      uint8_t aux_start, uint8_t aux_end,
      const uint8_t drive_cfg[6],
      void (* critical_callback)(BmsCriticalFrame_t),
      float (* uv_to_float)(uint16_t),
      float (* v_to_celsius)(float, float));

  ~BMS();

  void (*tick)();

  uint16_t * cell_codes;
  uint16_t * aux_codes;
  
  protected:
  LTC6804_2 * const ltc;
  IVT * const ivt;

  const uint8_t total_ic;
  const float ov, uv , ot, ut;
  const uint8_t cell_start, cell_end, aux_start, aux_end;
  
  const uint8_t * drive_cfg;

  void tick_charge_mode();
  void tick_drive_mode();
  
  void setup_charge_mode();
  void setup_drive_mode();
  
  void (* const critical_callback)(BmsCriticalFrame_t);
  float (* const uv_to_float)(uint16_t);
  float (* const v_to_celsius)(float, float);
};

#endif //FRAMEWORK_H
