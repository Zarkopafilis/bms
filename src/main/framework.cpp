/* Extensions and additions on LT_SPI and more
importantly LTC60842 functions to make them work for teensy. */

#include <Arduino.h>
#include "framework.h"
#include "config.h"

void output_low(uint8_t pin)
{
    digitalWrite(pin, LOW);
}

void output_high(uint8_t pin)
{
    digitalWrite(pin, HIGH);
}

void IVT::update(CAN_message_t message)
{
    uint32_t val = message.buf[2] << 24 | message.buf[3] << 16 | message.buf[4] << 8 | message.buf[5];
    float si = val * 0.001;

    switch(message.id){
        case IVT_CURRENT_CANID:
            this->old_amps = false;
            this->amps = si;
            break;
        case IVT_VOLTAGE_CANID:
            this->old_volts = false;
            this->volts = si;
            break;
        default:
#if DEBUG
            Serial.print("IVT Message ID: ");
            Serial.print(message.id);
            Serial.println(" not handled!");
#endif
            this->amps = 999;
            this->volts = 999;
            this->old_amps = true;
            this->old_volts = true;
    }
}

uint32_t const * IVT::get_ids(){ return this->ids; }
uint32_t IVT::get_id_num(){ return IVT::id_num; }

IVTMeasureFrame_t IVT::tick()
{
    if(this->old_amps || this->old_volts)
    {
        return {IVT_OLD_MEASUREMENT, this->amps, this->volts};
    }

    this->old_amps = true;
    this->old_volts = true;
    return {IVT_OLD_MEASUREMENT, this->amps, this->volts};
}

IVT_Dummy::IVT_Dummy(float amps, float volts) : amps(amps), volts(volts){}

IVTMeasureFrame_t IVT_Dummy::tick()
{
    return {IVT_SUCCESS, this->amps, this->volts};
}

void IVT_Dummy::update(CAN_message_t message) {}

BMS::BMS(LTC6804_2 * ltc, IVT * ivt,
         uint8_t total_ic,
         float overvolts,
         float undervolts,
         float overtemp,
         float undertemp,
         uint8_t cell_start, uint8_t cell_end,
         uint8_t aux_start, uint8_t aux_end,
         const uint8_t conf[6],
         void (* critical_callback)(BmsCriticalFrame_t),
         float (* const uv_to_float)(uint16_t),
         float (* const v_to_celsius)(float, float)) :
    ltc(ltc), ivt(ivt), total_ic(total_ic),
    ov(overvolts), uv(undervolts), ot(overtemp), ut(undertemp),
    cell_start(cell_start), cell_end(cell_end), aux_start(aux_start), aux_end(aux_end),
    config(conf),
    critical_callback(critical_callback), uv_to_float(uv_to_float), v_to_celsius(v_to_celsius)
{
    this->cell_codes = (uint16_t *) malloc(sizeof(uint16_t) * total_ic * (cell_end - cell_start));
    this->aux_codes = (uint16_t *) malloc(sizeof(uint16_t) * total_ic * (aux_end - aux_start + 1));

#if DEBUG
    Serial.println("Writing configuration to slaves");
#endif

    //Write new configuration to each slave
    //WRCFG (Write Configuration) Command
    uint8_t cfg[total_ic][6];

    for(uint8_t i = 0; i < total_ic; i++)
    {
        for(uint8_t j = 0; j < 6; j++)
        {
            cfg[i][j] = *(config + j);
        }
    }

    ltc->wakeup_sleep();
    ltc->wrcfg(total_ic, cfg);

#if DEBUG
    Serial.println("Evaluating wether configuration was indeed changed");
#endif

    uint8_t r_cfg[total_ic][8];
    //Read Configuration again to evaluate that previous step took effect
    //RDCFG Command
    int pec = ltc->rdcfg(total_ic, r_cfg);

    if(pec == -1)
    {
#if DEBUG_PEC
        Serial.print("Slaves failed to send correct configuration back!");
#endif
        critical_callback(bms_pec_error);
    }

    for(uint8_t addr = 0; addr < total_ic; addr++)
    {
        uint8_t check = 0xFF;

        //No need to check the 2 PEC bytes again
        for (uint8_t i = 0; i < 6; i++)
        {
            uint8_t b = r_cfg[addr][i];
            check &= (b == *(config + i));
        }

        if(check == 0)
        {
#if DEBUG
            Serial.print("Slave ");
            Serial.print(addr);
            Serial.println(" was not properly configured!");
#endif
            critical_callback(bms_critical_error);
        }
    }

#if DEBUG
    Serial.println("Clearing cell registers of all the slaves");
#endif
    //Broadcast cell clear
    //CLRCELL Command
    ltc->clrcell();

#if DEBUG
    Serial.println("Making sure no cell register bits are stuck");
#endif
    //Check if every cell is 0xFF in order to determine
    //Wether any possible bits are stuck

#if DEBUG
    Serial.println("Reading all cell values and comparing with 0xFF");
#endif
    //Start reading everything
    uint16_t cell_codez[total_ic][12];
    pec = ltc->rdcv(CELL_CH_ALL, total_ic, cell_codez);

    if(pec == -1)
    {
#if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to RDCV!");
#endif
        critical_callback(bms_pec_error);
    }

    for(uint8_t addr = 0; addr < total_ic; addr++)
    {
        for(uint8_t cell = cell_start; cell < cell_end; cell++)
        {
            if(cell_codez[addr][cell] != 0xFFFF)
            {
#if DEBUG
                Serial.print("Slave #");
                Serial.print(addr);
                Serial.print("'s cell #");
                Serial.print(cell);
                Serial.println(" may have got a bit stuck (not 0xFFFF)!");
#endif
                critical_callback(bms_critical_error);
            }
        }
    }

#if DEBUG
    Serial.println("Clearing auxiliary registers of all the slaves");
#endif
    //Broadcast auxiliary clear
    //CLRAUX Command
    ltc->clraux();

#if DEBUG
    Serial.println("Making sure no auxiliary register bits are stuck");
#endif
    //Check if every auxiliary register is 0xFF in order to determine
    //Wether any possible bits are stuck
    uint16_t aux_codez[total_ic][6];
    pec = ltc->rdaux(AUX_CH_ALL, total_ic, aux_codez);

    if(pec == -1)
    {
#if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to RDAUX!");
#endif
        critical_callback(bms_pec_error);
    }

    for(uint8_t addr = 0; addr < total_ic; addr++)
    {
        for(uint8_t aux = 0; aux < 6; aux++)
        {
            if(aux_codez[addr][aux] != 0xFFFF)
            {
#if DEBUG
                Serial.print("Slave #");
                Serial.print(addr);
                Serial.print("'s aux register #");
                Serial.print(aux);
                Serial.println(" may have got a bit stuck (not 0xFFFF)!");
#endif
                critical_callback(bms_pec_error);
            }
        }
    }

#if DEBUG
    Serial.println("Performing 1 measurement and ignoring the results");
#endif

    ltc->adcv();

    pec = ltc->rdcv(CELL_CH_ALL, total_ic, cell_codez);

    if(pec == -1)
    {
#if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to first demo measurement (cells)!");
#endif
        critical_callback(bms_pec_error);
    }

    ltc->adax();

    pec = ltc->rdaux(AUX_CH_ALL, total_ic, aux_codez);

    if(pec == -1)
    {
#if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to first demo measurement (aux/temp)!");
#endif
        critical_callback(bms_pec_error);
    }

#if DEBUG
    Serial.println("> Setup Complete...Starting in 2 Seconds!");
#endif

    //Ensure that we get back correct measurements from the battery monitors on tick
    delay(2000);
}

Float_Index_Tuple_t BMS::get_volts(bool greater){
  uint16_t target_volts = *(cell_codes);
  uint8_t index = 0;
  for(uint8_t slave = 0; slave < total_ic; slave++){
    for(uint8_t cell = cell_start; cell < cell_end; cell++){
        uint16_t volts = *(cell_codes + slave * (cell_end - cell_start) + cell);
        if((volts > target_volts) == greater){
          target_volts = volts;
          index = slave * (cell_start - cell_end) + cell;
        }
    }
  }
  return Float_Index_Tuple_t{ uv_to_float(target_volts) ,index};
}

Float_Index_Tuple_t BMS::get_temp(bool greater){
  uint16_t target_temp = *(aux_codes);
  uint16_t target_vref = *(aux_codes + 5);
  uint8_t index = 0;
  for(uint8_t slave = 0; slave < total_ic; slave++){
    uint16_t vref = *(aux_codes + slave * (aux_end - aux_start) + 5);
    for(uint8_t aux = aux_start; aux < aux_end; aux++){  
        uint16_t temp = *(aux_codes + slave * (aux_end - aux_start) + aux);
        if((temp > target_temp) == greater){
          target_temp = temp;
          target_vref = vref;
          index = slave * (aux_end - aux_start) + aux;
        }
    }
  }
  return Float_Index_Tuple_t{ v_to_celsius(uv_to_float(target_temp), uv_to_float(target_vref)) ,index};
}

Float_Index_Tuple_t BMS::get_min_volts(){ return get_volts(false); }
Float_Index_Tuple_t BMS::get_max_volts(){ return get_volts(true); }
 
Float_Index_Tuple_t BMS::get_min_temp(){ return get_temp(false); }
Float_Index_Tuple_t BMS::get_max_temp(){ return get_temp(true); }


void BMS::set_cfg(const uint8_t cfg[6])
{
    this->config = cfg;
}

BMS::~BMS()
{
    free(this->aux_codes);
    free(this->cell_codes);
}

void BMS::tick()
{
    uint16_t cell_codez[total_ic][12];
    uint16_t aux_codez[total_ic][6];

    //Write configuration to each slave
    uint8_t cfg[total_ic][6];

    for(uint8_t i = 0; i < total_ic; i++)
    {
        for(uint8_t j = 0; j < 6; j++)
        {
            cfg[i][j] = *(config + j);
        }
    }

    //Write the configuration on every slave on each loop because it gets lost after some time
    ltc->wakeup_sleep();
    ltc->wrcfg(total_ic, cfg);

    //Transmit Analog-Digital Conversion Start Broadcast to measure CELLS
    //ADCV Command
    ltc->adcv();

    //Start reading everything back
    int pec = ltc->rdcv(CELL_CH_ALL, total_ic, cell_codez);

    if(pec == -1)
    {
#if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to RDCV!");
#endif
        critical_callback(bms_critical_error);
    }

    for(uint8_t addr = 0; addr < total_ic; addr++)
    {
        for(uint8_t cell = cell_start; cell < cell_end; cell++)
        {
#if DEBUG_CELL_VALUES
            Serial.print("Slave #");
            Serial.print(addr);
            Serial.print("'s cell #");
            Serial.print(cell);
            Serial.print(" -> ");
            Serial.print(uv_to_float(cell_codez[addr][cell]),4);
            Serial.println(" V");
#endif
        }
    }

    //Transmit Analog-Digital Conversion Start Broadcast to measure GPIOs (Auxiliary)
    //ADAX Command
    ltc->adax();

    //Read GPIO Volts (Temperature values here)
    //RDAUX Command (GPIO Measurements are stored in auxiliary registers)
    pec = ltc->rdaux(AUX_CH_ALL, total_ic, aux_codez);

    if(pec == -1)
    {
#if DEBUG_PEC
        Serial.println("Slaves sent back incorrect response to RDAUX!");
#endif
        critical_callback(bms_critical_error);
    }

    for(uint8_t addr = 0; addr < total_ic; addr++)
    {
        uint16_t vref = aux_codez[addr][5];
#if DEBUG_CELL_VALUES
        Serial.print("VRef2 -> ");
        Serial.print(uv_to_float(vref), 4);
        Serial.println(" V");
#endif
        for(uint8_t temp = aux_start; temp < aux_end; temp++)
        {
#if DEBUG_CELL_VALUES
            Serial.print("Slave #");
            Serial.print(addr);
            Serial.print("'s GPIO #");
            Serial.print(temp);
            Serial.print(" -> ");
            Serial.print(v_to_celsius(uv_to_float(aux_codez[addr][temp]), uv_to_float(vref)), 4);
            Serial.print(" C <=> ");
            Serial.print(uv_to_float(aux_codez[addr][temp]), 4);
            Serial.println(" V");
#endif
        }
    }

    //After Volts and Temps, read stuff from the current Can_Sensor
    IVTMeasureFrame_t current_frame = this->ivt->tick();

    if(current_frame.success == IVT_SUCCESS)
    {
#if DEBUG_CURRENT_VALUES
        Serial.print("IVT Current: ");
        Serial.print(current_frame.amps);
        Serial.println(" A");
#endif

    }
    else
    {
#if DEBUG_CURRENT_VALUES
        if(current_frame.success == IVT_OLD_MEASUREMENT){
          Serial.println("IVT Current sensor hasn't sent new measurement yet! Possible sensor loss!");
        }else{
          Serial.println("IVT Current sensor loss!");
        }
#endif
        critical_callback(bms_current_error);
    }

    //Defensively copy the measurements just so they can be read only
    for(uint8_t addr = 0; addr < total_ic; addr++)
    {
        for(uint8_t cell = 0; cell < (cell_end - cell_start); cell++)
        {
            *(cell_codes + addr * (cell_end - cell_start) + cell) = cell_codez[addr][cell + cell_start];
        }
    }
    for(uint8_t addr = 0; addr < total_ic; addr++)
    {
        for(uint8_t temp = 0; temp < (aux_end - aux_start + 1); temp++) // +1 to include VRef
        {
            *(this->aux_codes + addr * (aux_end - aux_start + 1) + temp) = aux_codez[addr][temp + aux_start];
        }
    }
}

CAN_message_t Liion_Bms_Can_Adapter::VoltageMinMax(BMS * bms){
  CAN_message_t msg;
  msg.id = LIION_START_CANID + LIION_VOLT_MIN_MAX_OFFSET;

  Float_Index_Tuple_t min = bms->get_min_volts();
  
  msg.buf[2] = min.index;
  msg.buf[3] = ((uint16_t)min.value * 100) & 0xFF; // Set to mV and keep only 8 LSBs
  
  return msg;
}

