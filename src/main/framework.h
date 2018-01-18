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

#ifndef IVT_CURRENT_CANID
#define IVT_CURRENT_CANID 0x521
#endif

#ifndef IVT_VOLTAGE_CANID
#define IVT_VOLTAGE_CANID 0x522
#endif

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
    float volts;
    float temp;
    uint32_t amps;
} BmsCriticalFrame_t;

static constexpr BmsCriticalFrame_t bms_critical_error{-10, -1, -1, 0xFF};
static constexpr BmsCriticalFrame_t bms_pec_error{-1, -1, -1, 0xFF};
static constexpr BmsCriticalFrame_t bms_current_error{-2, 0, 0, 0xFF};

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

    uint8_t const * config;

    void (* const critical_callback)(BmsCriticalFrame_t);
    float (* const uv_to_float)(uint16_t);
    float (* const v_to_celsius)(float, float);
};

#endif //FRAMEWORK_H
