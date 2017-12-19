/*!
DC1942B
LTC6804-2: Battery stack monitor

@verbatim
NOTES
 Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.
   Ensure all jumpers on the demo board are installed in their default positions from the factory.
   Refer to Demo Manual D1894B.
 

 Menu Entry 1: Write Configuration
   Writes the configuration register of the LTC6804. This command can be used to turn on the reference. 
   
 Menu Entry 2: Read Configuration
   Reads the configuration register of the LTC6804, the read configuration can differ from the written configuration.
   The GPIO pins will reflect the state of the pin

 Menu Entry 3: Start Cell voltage conversion
    Starts a LTC6804 cell channel adc conversion.

 Menu Entry 4: Read cell voltages
    Reads the LTC6804 cell voltage registers and prints the results to the serial port.
 
 Menu Entry 5: Start Auxiliary voltage conversion
    Starts a LTC6804 GPIO channel adc conversion.

 Menu Entry 6: Read Auxiliary voltages
    Reads the LTC6804 axiliary registers and prints the GPIO voltages to the serial port.
 
 Menu Entry 7: Start cell voltage measurement loop
    The command will continuously measure the LTC6804 cell voltages and print the results to the serial port.
    The loop can be exited by sending the MCU a 'm' character over the serial link.
 
USER INPUT DATA FORMAT:
 decimal : 1024
 hex     : 0x400
 octal   : 02000  (leading 0)
 binary  : B10000000000
 float   : 1024.0
 
@endverbatim

REVISION HISTORY
$Revision: 1000 $
$Date: 2013-12-13 

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2013 Linear Technology Corp. (LTC)
 */

 
/*! @file 
    @ingroup LTC68042 
*/  

#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "LT_SPI.h"
#include "LTC68042.h"
#include <SPI.h>

const uint8_t TOTAL_IC = 1;//!<number of ICs in the isoSPI network LTC6804-2 ICs must be addressed in ascending order starting at 0.

/******************************************************
 *** Global Battery Variables received from 6804 commands
 These variables store the results from the LTC6804
 register reads and the array lengths must be based 
 on the number of ICs on the stack
 ******************************************************/
uint16_t cell_codes[TOTAL_IC][12]; 
/*!< 
  The cell codes will be stored in the cell_codes[][12] array in the following format:
  
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

uint16_t aux_codes[TOTAL_IC][6];
/*!<
 The GPIO codes will be stored in the aux_codes[][6] array in the following format:
 
 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/

uint8_t tx_cfg[TOTAL_IC][6];
/*!<
  The tx_cfg[][6] stores the LTC6804 configuration data that is going to be written 
  to the LTC6804 ICs on the daisy chain. The LTC6804 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:
  
 |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |
 
*/

uint8_t rx_cfg[TOTAL_IC][8];
/*!<
  the rx_cfg[][8] array stores the data that is read back from a LTC6804-1 daisy chain. 
  The configuration data for each IC  is stored in blocks of 8 bytes. Below is an table illustrating the array organization:

|rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
|---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
|IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/

/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()                  
{
  Serial.begin(115200);
  LTC6804_initialize();  //Initialize LTC6804 hardware
  init_cfg();        //initialize the 6804 configuration array to be written
  print_menu();              
}

/*!*********************************************************************
  \brief main loop

***********************************************************************/
void loop()                     
{
  
  if (Serial.available())           // Check for user input
    {
      uint32_t user_command;
      user_command = read_int();      // Read the user command
      Serial.println(user_command);
      run_command(user_command);
    }
}


/*!*****************************************
  \brief executes the user inputted command
  
  Menu Entry 1: Write Configuration \n
   Writes the configuration register of the LTC6804. This command can be used to turn on the reference 
   and increase the speed of the ADC conversions. 
   
 Menu Entry 2: Read Configuration \n
   Reads the configuration register of the LTC6804, the read configuration can differ from the written configuration.
   The GPIO pins will reflect the state of the pin

 Menu Entry 3: Start Cell voltage conversion \n
   Starts a LTC6804 cell channel adc conversion.

 Menu Entry 4: Read cell voltages
    Reads the LTC6804 cell voltage registers and prints the results to the serial port.
 
 Menu Entry 5: Start Auxiliary voltage conversion
    Starts a LTC6804 GPIO channel adc conversion.

 Menu Entry 6: Read Auxiliary voltages
    Reads the LTC6804 axiliary registers and prints the GPIO voltages to the serial port.
 
 Menu Entry 7: Start cell voltage measurement loop
    The command will continuously measure the LTC6804 cell voltages and print the results to the serial port.
    The loop can be exited by sending the MCU a 'm' character over the serial link.
 
*******************************************/
void run_command(uint16_t cmd)
{
  int8_t error = 0;
  float t;
  float current_time1=0,current_time2=0;
  
  char input = 0;
  switch(cmd)
  {
   
  case 1:
    wakeup_sleep();
    LTC6804_wrcfg(TOTAL_IC,tx_cfg);
    print_config();
    break;
    
  case 2:
    wakeup_sleep();
    error = LTC6804_rdcfg(TOTAL_IC,rx_cfg);
    if (error == -1)
    {
     Serial.println("A PEC error was detected in the received data");
    }
    print_rxconfig();
    break;

  case 3:
    wakeup_sleep();
    LTC6804_adcv();
    delay(3);
    Serial.println("cell conversion completed");
    Serial.println();
    break;
    
  case 4:
    wakeup_sleep();
    error = LTC6804_rdcv(0, TOTAL_IC,cell_codes); // Set to read back all cell voltage registers
    if (error == -1)
    {
       Serial.println("A PEC error was detected in the received data");
    }
    print_cells();
    break;
    
  case 5:
    wakeup_sleep();
    LTC6804_adax();
    delay(3);
    Serial.println("aux conversion completed");
    Serial.println();
    break;
    
  case 6:
    wakeup_sleep();
    write_tx_cfg();
    error = LTC6804_rdaux(0,TOTAL_IC,aux_codes); // Set to read back all aux registers
    if (error == -1)
    {
      Serial.println("A PEC error was detected in the received data");
    }
    t=convert_C();
    Serial.println(t);
    break;
  
  case 7:
    Serial.println("transmit 'm' to quit");
    wakeup_sleep();
    LTC6804_wrcfg(TOTAL_IC,tx_cfg);
    while (input != 'm')
    {
      if (Serial.available() > 0)
      {
        input = read_char();
      }
      wakeup_idle();
      LTC6804_adcv();
      delay(10);
      wakeup_idle();
      error = LTC6804_rdcv(0, TOTAL_IC,cell_codes);
      if (error == -1)
      {
       Serial.println("A PEC error was detected in the received data");
      }
      print_cells();
      delay(500);
    }
    print_menu();
    break;  

  case 8:
    write_tx_cfg();
    wakeup_sleep();
    LTC6804_wrcfg(TOTAL_IC,tx_cfg);
    
    while (input != 'm'){
    
       if (Serial.available() > 0)
      {
        input = read_char();
      }
      wakeup_sleep();
      LTC6804_adcvax();
      delay(3);
      //Serial.println("aux conversion completed");
      //Serial.println();
      
      wakeup_sleep();
      error = LTC6804_rdaux(1,TOTAL_IC,aux_codes); // Set to read back all aux registers
      if (error == -1)
      {
        Serial.println("A PEC error was detected in the received data");
      }
      
      error = LTC6804_rdcv(1, TOTAL_IC,cell_codes);
      if (error == -1)
      {
       Serial.println("A PEC error was detected in the received data");
      }

      print_cells();
      t=convert_C();
      
      Serial.println(t);
      Serial.println("---------------------------------------------------------");
      Serial.println();
      
      delay(500);
    }
    break;

    case 9:
      
      write_tx_cfg();
      Serial.println("The cfgr[] has changed\n");
      //wakeup_sleep();
      Serial.println("tx_cfg has been sent to be written");
      LTC6804_wrcfg(TOTAL_IC,tx_cfg);
      current_time1=millis();
      //wakeup_sleep();
      error = LTC6804_rdcfg(TOTAL_IC,rx_cfg);
      if (error == -1){ 
        Serial.println("A PEC error was detected in the received data");
      }
      else Serial.println("The registers have been written SUCCESSFULLY!");
      current_time2=millis();
      Serial.print("Discharge has started ");
      Serial.print((current_time2-current_time1),5);
      Serial.println(" milli seconds before (aprox)");
      
      //delay(10000);

      //Serial.println("After a delay of 55000 msecs the iniated tx_cfg[] will be sent");
      //init_cfg();
      wakeup_sleep();
      input = read_char();
      if (input=='m'){
        init_cfg();
        LTC6804_wrcfg(TOTAL_IC,tx_cfg);
        
      }
      
      //wakeup_sleep();
     // Serial.println("tx_cfg has changed");
      
     // Serial.println("tx_cfg has been sent to be written");
     /* wakeup_sleep();
      error = LTC6804_rdcfg(TOTAL_IC,rx_cfg);
      if (error == -1){ 
        Serial.println("A PEC error was detected in the received data");
      }
      else Serial.println("The registers have been written SUCCESSFULLY!");
      //print_rxconfig();*/
      break;


  default:
     Serial.println("Incorrect Option");
     break; 
  }
}

/*!***********************************
 \brief Initializes the configuration array
 **************************************/
void init_cfg()
{
  for(int i = 0; i<TOTAL_IC;i++)
  {
    tx_cfg[i][0] = 0xFE;
    tx_cfg[i][1] = 0x00; 
    tx_cfg[i][2] = 0x00;
    tx_cfg[i][3] = 0x00; 
    tx_cfg[i][4] = 0x00;
    tx_cfg[i][5] = 0x00;
  }
}

//Used only for testing! Normal config is above, at init_cfg()
void write_tx_cfg()
{
  for (int i = 0; i<TOTAL_IC; i++)
  {
    tx_cfg[i][0] = B01011110;
    tx_cfg[i][1] = B00000000;
    tx_cfg[i][2] = B00000000;
    tx_cfg[i][3] = B00000000;
    tx_cfg[i][4] = B11111111;//Discharge every single cell(TEST)
    tx_cfg[i][5] = B00011111;
  }
}

/*!*********************************
  \brief Prints the main menu 
***********************************/
void print_menu()
{
  Serial.println("Please enter LTC6804 Command");
  Serial.println("Write Configuration: 1");
  Serial.println("Read Configuration: 2");
  Serial.println("Start Cell Voltage Conversion: 3");
  Serial.println("Read Cell Voltages: 4");
  Serial.println("Start Aux Voltage Conversion: 5");
  Serial.println("Read Aux Voltages: 6");
  Serial.println("loop cell voltages: 7");
  Serial.println("Please enter command: ");
   Serial.println();
}



/*!************************************************************
  \brief Prints Cell Voltage Codes to the serial port
 *************************************************************/
void print_cells()
{

  
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic+1,DEC);
    for(int i=0; i<12; i++)
    {
      if(i!=0 && i!=1){
      Serial.print(" C");
      Serial.print(i+1,DEC);
      Serial.print(":");
      Serial.print(cell_codes[current_ic][i]*0.0001,4);
      Serial.print(",");
      }
    }
     Serial.println(); 
  }
    Serial.println(); 
}

/*!****************************************************************************
  \brief Prints GPIO Voltage Codes and Vref2 Voltage Code onto the serial port
 *****************************************************************************/
void print_aux()
{
  
  //for(int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  //{
    //Serial.print(" IC ");
    //Serial.print(current_ic+1,DEC);
    //for(int i=0; i < 5; i++)
    //{
     // Serial.print(" GPIO-");
      //Serial.print(i+1,DEC);
      //Serial.print(":");
      Serial.print(aux_codes[0][0]*0.01,2);
      //Serial.print(",");
    //}
     //Serial.print(" Vref2");
     //Serial.print(":");
     //Serial.print(aux_codes[current_ic][5]*0.0001,4);
     //Serial.println();
  //}
  //Serial.println(); 
}
/*!******************************************************************************
 \brief Prints the Configuration data that is going to be written to the LTC6804
 to the serial port.
 ********************************************************************************/
void print_config()
{
  int cfg_pec;
  
  Serial.println("Written Configuration: ");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic+1,DEC);
    Serial.print(": ");
    Serial.print("0x");
    serial_print_hex(tx_cfg[current_ic][0]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][1]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][2]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][3]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][4]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][5]);
    Serial.print(", Calculated PEC: 0x");
    cfg_pec = pec15_calc(6,&tx_cfg[current_ic][0]);
    serial_print_hex((uint8_t)(cfg_pec>>8));
    Serial.print(", 0x");
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println(); 
  }
   Serial.println(); 
}

/*!*****************************************************************
 \brief Prints the Configuration data that was read back from the 
 LTC6804 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
  Serial.println("Received Configuration ");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic+1,DEC);
    Serial.print(": 0x");
    serial_print_hex(rx_cfg[current_ic][0]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][1]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][2]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][3]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][4]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][5]);
    Serial.print(", Received PEC: 0x");
    serial_print_hex(rx_cfg[current_ic][6]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][7]);
    Serial.println(); 
  }
   Serial.println(); 
}

void serial_print_hex(uint8_t data)
{
    if (data< 16)
    {
      Serial.print("0");
      Serial.print((byte)data,HEX);
    }
    else
      Serial.print((byte)data,HEX);
}

float convert_C(){

  float R10=10;
  float R25=10;
  float vr,T,r;
  float Aa=0.003354016 ;
  float Bb=0.000256985 ;
  float Cc= 2.62013*0.000001;
  float Dd=6.38309*0.00000001;
  float t;
  
  vr =aux_codes[0][0]*0.0001;
  
  r=-(vr*R10)/(vr-aux_codes[0][5]*0.0001);
  T= Aa + Bb*log(r/R25)+ Cc*log(r/R25)*log(r/R25) +Dd*log(r/R25)*log(r/R25)*log(r/R25);

  t=1/T -272.15;
  return t;
}
