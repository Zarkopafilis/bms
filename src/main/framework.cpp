/* Extensions and additions on LT_SPI and more
importantly LTC60842 functions to make them work for teensy. */

#include <Arduino.h>
#include "framework.h"

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

float BMS::get_total_voltage(){
    float total_volts = 0;
    for(uint8_t addr = 0; addr < total_ic; addr++)
    {
        for(uint8_t cell = 0; cell < (cell_end - cell_start); cell++)
        {
            total_volts += uv_to_float(*(cell_codes + addr * (cell_end - cell_start) + cell));
        }
    }
    return total_volts;
}

CAN_message_t Liion_Bms_Can_Adapter::VoltageMinMax(BMS * bms){
  CAN_message_t msg;
  msg.id = LIION_START_CANID + LIION_VOLT_MIN_MAX_OFFSET;
  msg.len = 8;

  Float_Index_Tuple_t min = bms->get_min_volts();
  
  msg.buf[2] = min.index;
  msg.buf[3] = ((uint16_t) (bms->uv_to_float(min.value) *  10)); // Set to 100mV resolution
  
  return msg;
}

Charger::Charger(FlexCAN * can, uint16_t initial_volts, uint16_t initial_amps) : can(can), volts(initial_volts), amps(initial_amps) {}

void Charger::send_charge_message(){
  CAN_message_t msg;
  msg.id = CHARGER_COMMAND_CANID;
  msg.len = 7;
  
  msg.buf[0] = 0x80;
  msg.buf[1] = 0x01;
  msg.buf[2] = 0xF4;

  //Volts (SI resolution)
  msg.buf[3] = (volts >> 8) & 0xFF;
  msg.buf[4] = volts & 0xFF;

  //Amps (0.1 resolution)
  uint16_t send_amps = amps * 10;
  msg.buf[5] = (send_amps >> 8) & 0xFF;
  msg.buf[6] = send_amps & 0xFF;

  this->can->write(msg);
}

void Charger::set_volts(uint16_t v){ this->volts = v; }
void Charger::set_amps(uint16_t a){ this->amps = a; }
void Charger::set_volts_amps(uint16_t v, uint16_t a)
{
  set_volts(v);
  set_amps(a);  
}

Charger_Dummy::Charger_Dummy() : Charger(nullptr, 0 , 0){}

void Charger_Dummy::send_charge_message(){}

CAN_message_t Shutdown_Message_Factory::simple(uint8_t error){
  CAN_message_t msg;
  msg.id = SHUTDOWN_ERROR_CANID;
  msg.len = 8;

  msg.buf[7] = error + ERROR_OFFSET;
  msg.buf[6] = 0;
  msg.buf[5] = 0;
  msg.buf[4] = 0;
  msg.buf[3] = 0;
  msg.buf[2] = 0;
  msg.buf[1] = 0;
  msg.buf[0] = 0;
  return msg;
}

CAN_message_t Shutdown_Message_Factory::data(uint8_t error, uint32_t data){
  CAN_message_t msg;
  msg.id = SHUTDOWN_ERROR_CANID;
  msg.len = 5;

  msg.buf[7] = error + ERROR_OFFSET;
  
  msg.buf[6] = (data >> 24) & 0xFF;
  msg.buf[5] = (data >> 16) & 0xFF;
  msg.buf[4] = (data >> 8) & 0xFF;
  msg.buf[3] = (data) & 0xFF;
  msg.buf[2] = 0;
  msg.buf[1] = 0;
  msg.buf[0] = 0;
  
  return msg;
}

CAN_message_t Shutdown_Message_Factory::full(uint8_t error, uint32_t data, uint8_t index){
  CAN_message_t msg;
  msg.id = SHUTDOWN_ERROR_CANID;
  msg.len = 8;

  msg.buf[7] = error + ERROR_OFFSET;
  
  msg.buf[6] = (data >> 24) & 0xFF;
  msg.buf[5] = (data >> 16) & 0xFF;
  msg.buf[4] = (data >> 8) & 0xFF;
  msg.buf[3] = (data) & 0xFF;

  msg.buf[2] = index;
  msg.buf[1] = 0;
  msg.buf[0] = 0;
  
  return msg;
}

Other_Battery_Box::Other_Battery_Box(FlexCAN * can) : can(can) {}

void Other_Battery_Box::update(CAN_message_t message){
  if(message.id == OTHER_BOX_VOLTAGE_CANID){
    float volts = (message.buf[7] << 8 | message.buf[6]) * 0.01;
    this->volts = volts;
  }
}

void Other_Battery_Box::send_total_voltage(float volts){
  CAN_message_t msg;
  msg.id = CURRENT_BOX_VOLTAGE_CANID;
  msg.len = 8;

  uint16_t v = volts * 100;

  msg.buf[7] = (v >> 8) & 0xFF;
  msg.buf[6] = (v) & 0xFF;
  msg.buf[5] = 0;
  msg.buf[4] = 0;
  msg.buf[3] = 0;
  msg.buf[2] = 0;
  msg.buf[1] = 0;
  msg.buf[0] = 0;
  can->write(msg);
}

float Other_Battery_Box::get_volts(){ return this->volts; }

uint32_t const * Other_Battery_Box::get_ids(){ return this->ids; }
uint32_t Other_Battery_Box::get_id_num(){ return this->id_num; }

Configurator::Configurator(FlexCAN * can) : can(can) {}

void Configurator::update(CAN_message_t message){
  if(message.id == CONFIGURATION_CANID){
    uint8_t addr = message.buf[7];
    uint8_t num = message.buf[6];

    if(addr == 0xFF){
      EEPROM.write(CONFIG_ADDRESS_VALIDITY, 0xFF);
    }else if(addr < CONFIG_ADDRESS_START || addr > EEPROM.length() || num > 6){
      return; 
    }else{
      for(uint8_t i = 0; i < num; i++){
        EEPROM.write(addr + i, message.buf[5 + i]);
      }
    }

    CAN_message_t ack;
    ack.id = CONFIGURATION_ACK_CANID;
    ack.len = 1;
    
    ack.buf[0] = BOX_ID * 32;

    can->write(ack);
  }
}
  
uint32_t const * Configurator::get_ids(){ return this->ids; }
uint32_t Configurator::get_id_num(){ return this->id_num; }

