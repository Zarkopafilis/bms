/*LTC6804-2 Multicell Battery Monitor Library*/
#include <Arduino.h>
#include "LT_SPI.h"
#include "LTC68042.h"
#include "config.h"
#include "framework.h"

/*Maps  global ADC control variables to the appropriate control bytes for each of the different ADC commands
 
@MD The adc conversion mode
@DCP Controls if Discharge is permitted during cell conversions
@CH Determines which cells are measured during an ADC conversion command
@CHG Determines which GPIO channels are measured during Auxiliary conversion command
 
 Command Code:
      |command  |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   | 
      |-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
      |ADCV:      |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] | 
      |ADAX:      |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|*/
void LTC6804_2::set_adc(uint8_t md, uint8_t dcp, uint8_t ch, uint8_t chg)
{
  uint8_t md_bits;
  
  md_bits = (md & 0x02) >> 1;
  ADCV[0] = md_bits + 0x02;
  md_bits = (md & 0x01) << 7;
  ADCV[1] =  md_bits + 0x60 + (dcp<<4) + ch;
 
  md_bits = (md & 0x02) >> 1;
  ADAX[0] = md_bits + 0x04;
  md_bits = (md & 0x01) << 7;
  ADAX[1] = md_bits + 0x60 + chg ;
 }

/*This function will initialize all 6804 variables and the SPI port.
Input: IC: number of ICs being controlled. The address of the ICs in a LTC6804-2 network will start at 0 and continue in an ascending order.*/
LTC6804_2::LTC6804_2(LT_SPI * lt_spi,
                      uint8_t total_ic,
                      uint8_t adc_conversion_mode,
                      uint8_t dishcharge_mode,
                      uint8_t cell_channels,
                      uint8_t aux_channels,
                      uint8_t delay_on_send_ms) : spi(lt_spi), total_ic(total_ic), delay_on_send_ms(delay_on_send_ms)
{
  //Fastest conversion mode - Disabled Discharge - AUX_CH_ALL measures all 5 GPIOs and 2nd Vref 
  set_adc(adc_conversion_mode,DCP_MODE,CELL_CH_ALL,AUX_CH_ALL);
}

/*Starts cell voltage conversion
  
  Starts ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion done is set using the associated global variables:
 |Variable|Function                                      | 
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted	     |*/
void LTC6804_2::adcv()
{
  uint8_t cmd[4];
  uint16_t temp_pec;
  
  //1
  cmd[0] = ADCV[0];
  cmd[1] = ADCV[1];
  
  //2
  temp_pec = LTC6804_2::pec15_calc(2, ADCV);
  cmd[2] = (uint8_t)(temp_pec >> 8);
  cmd[3] = (uint8_t)(temp_pec);
  
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  
  //4
  output_low(this->spi->cs);
  this->spi_write_array(4,cmd);
  output_high(this->spi->cs);

  delay(delay_on_send_ms);
}

void LTC6804_2::adcvax()
{
  uint8_t cmd[4];

  //-----If MD or DCP changes the the cmd[] array will change-----
  cmd[0] = 0x05;
  cmd[1] = 0x6F;
  
  cmd[2] = 0x9C;
  cmd[3] = 0x54; 
 
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  output_low(this->spi->cs);
  this->spi_write_array(4,cmd);
  output_high(this->spi->cs);

  delay(delay_on_send_ms);
}

/*
  LTC6804_adcv Function sequence:
  1. Load adcv command into cmd array
  2. Calculate adcv cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast adcv command to LTC6804 stack*/

/*Start an GPIO Conversion
 
  Starts an ADC conversions of the LTC6804 GPIO inputs.
  The type of ADC conversion done is set using the associated global variables:
 |Variable|Function                                      | 
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |*/
void LTC6804_2::adax()
{
  uint8_t cmd[4];
  uint16_t temp_pec;
 
  cmd[0] = ADAX[0];
  cmd[1] = ADAX[1];
  temp_pec = LTC6804_2::pec15_calc(2, ADAX);
  cmd[2] = (uint8_t)(temp_pec >> 8);
  cmd[3] = (uint8_t)(temp_pec);
 
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  output_low(this->spi->cs);
  this->spi_write_array(4,cmd);
  output_high(this->spi->cs);

  delay(delay_on_send_ms);
}
/*LTC6804_adax Function sequence:
  
  1. Load adax command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast adax command to LTC6804 stack*/


/*Reads and parses the LTC6804 cell voltage registers.
 
 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable. 
 
@param[in] uint8_t reg; This controls which cell voltage register is read back. 
          0: Read back all Cell registers 
          1: Read back cell group A 
          2: Read back cell group B 
          3: Read back cell group C 
          4: Read back cell group D 

@param[in] uint8_t total_ic; This is the number of ICs in the network
 
@param[out] uint16_t cell_codes[]; An array of the parsed cell codes from lowest to highest. The cell codes will
  be stored in the cell_codes[] array in the following format:
  |  cell_codes[0]| cell_codes[1] |  cell_codes[2]|    .....     |  cell_codes[11]|  cell_codes[12]| cell_codes[13] |  .....   |
  |---------------|----------------|--------------|--------------|----------------|----------------|----------------|----------|
  |IC1 Cell 1     |IC1 Cell 2      |IC1 Cell 3    |    .....     |  IC1 Cell 12   |IC2 Cell 1      |IC2 Cell 2      | .....    |
 
 @return int8_t, PEC Status:
	0: No PEC error detected
	-1: PEC error detected, retry read*/
uint8_t LTC6804_2::rdcv(uint8_t reg, uint16_t cell_codes[][12])
{
  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;
  
  uint8_t *cell_data;
  int8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter=0; //data counter
  cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
  //1.a
  if (reg == 0)
  {
    //a.i
    for(uint8_t cell_reg = 1; cell_reg<5; cell_reg++)         			 //executes once for each of the LTC6804 cell voltage registers
    {
      data_counter = 0;
      this->rdcv_reg(cell_reg, cell_data);
      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++) // executes for every LTC6804 in the stack
      {																 	  // current_ic is used as an IC counter
        //a.ii
		for(uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)	 								  // This loop parses the read back data. Loops 
        {														   		  // once for each cell voltages in the register 
          parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);
          cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
          data_counter = data_counter + 2;
        }
		//a.iii
        received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1];
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT ]);
        if(received_pec != data_pec)
        {
          pec_error--;//pec_error = -1;
        }
        data_counter=data_counter+2;
      }
    }
  }
 //1.b
  else
  {
	//b.i
	
    this->rdcv_reg(reg, cell_data);
    for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++) // executes for every LTC6804 in the stack
    {							   									// current_ic is used as an IC counter
		//b.ii
		for(uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)   									// This loop parses the read back data. Loops 
		{						   									// once for each cell voltage in the register 
			parsed_cell = cell_data[data_counter] + (cell_data[data_counter+1]<<8);
			cell_codes[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] = 0x0000FFFF & parsed_cell;
			data_counter= data_counter + 2;
		}
		//b.iii
	    received_pec = (cell_data[data_counter] << 8 )+ cell_data[data_counter + 1];
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT * (reg-1)]);
		if(received_pec != data_pec)
		{
			pec_error--;//pec_error = -1;
		}
	}
  }
 free(cell_data);
 //2
return(pec_error);
}
/*LTC6804_rdcv Sequence
	1. Switch Statement:
		a. Reg = 0
			i. Read cell voltage registers A-D for every IC in the stack
			ii. Parse raw cell voltage data in cell_codes array
			iii. Check the PEC of the data read back vs the calculated PEC for each read register command
		b. Reg != 0 
			i.Read single cell voltage register for all ICs in stack
			ii. Parse raw cell voltage data in cell_codes array
			iii. Check the PEC of the data read back vs the calculated PEC for each read register command
	2. Return pec_error flag*/


/*Read the raw data from the LTC6804 cell voltage register
 
 The function reads a single cell voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of 
 the LTC6804_rdcv() command. 
 
 @param[in] uint8_t reg; This controls which cell voltage register is read back. 
          1: Read back cell group A 
          2: Read back cell group B 
          3: Read back cell group C 
          4: Read back cell group D 
		  
 @param[in] uint8_t total_ic; This is the number of ICs in the network
 
 @param[out] uint8_t *data; An array of the unparsed cell codes*/
void LTC6804_2::rdcv_reg(uint8_t reg, uint8_t *data)
{
  uint8_t cmd[4];
  uint16_t temp_pec;
  
  //1
  if (reg == 1)
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if(reg == 2)
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  } 
  else if(reg == 3)
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  } 
  else if(reg == 4)
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  } 
  //2, 3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  
  //4
  for(int current_ic = 0; current_ic<total_ic; current_ic++)
  {
	cmd[0] = 0x80 + (current_ic<<3); //Setting address
    temp_pec = LTC6804_2::pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(temp_pec >> 8);
	cmd[3] = (uint8_t)(temp_pec); 
	output_low(this->spi->cs);
	this->spi_write_read(cmd,4,&data[current_ic*8],8);
	output_high(this->spi->cs);
  }
}
/*LTC6804_rdcv_reg Function Process:
  1. Determine Command and initialize command array
  2. Calculate Command PEC
  3. Wake up isoSPI, this step is optional
  4. Send Global Command to LTC6804 stack*/


/*Reads and parses the LTC6804 auxiliary registers.
 
 The function is used
 to read the  parsed GPIO codes of the LTC6804. This function will send the requested 
 read commands parse the data and store the gpio voltages in aux_codes variable 
 
 @param[in] uint8_t reg; This controls which GPIO voltage register is read back. 
          0: Read back all auxiliary registers 
          1: Read back auxiliary group A 
          2: Read back auxiliary group B 
 
 @param[in] uint8_t total_ic; This is the number of ICs in the network

 @param[out] uint8_t aux_codes[]; An array of the aux codes from lowest to highest. The GPIO codes will
 be stored in the aux_codes[] array in the following format:
 |  aux_codes[0]| aux_codes[1] |  aux_codes[2]|  aux_codes[3]|  aux_codes[4]|  aux_codes[5]| aux_codes[6] |aux_codes[7]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|------------|-----------|
 |IC1 GPIO1     |IC1 GPIO2     |IC1 GPIO3     |IC1 GPIO4     |IC1 GPIO5     |IC1 Vref2     |IC2 GPIO1     |IC2 GPIO2   |  .....    |
 
 @return int8_t, PEC Status.
	0: No PEC error detected
	-1: PEC error detected, retry read*/
int8_t LTC6804_2::rdaux(uint8_t reg, uint16_t aux_codes[][6])
{
  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t GPIO_IN_REG = 3;
  
  uint8_t *data;
  uint8_t data_counter = 0; 
  int8_t pec_error = 0;
  uint16_t received_pec;
  uint16_t data_pec;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
  //1.a
  if (reg == 0)
  {
	//a.i
    for(uint8_t gpio_reg = 1; gpio_reg<3; gpio_reg++)		 	   		 //executes once for each of the LTC6804 aux voltage registers
    {
      data_counter = 0;
      this->rdaux_reg(gpio_reg, data);
      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++) // This loop executes once for each LTC6804
      {									  								 // current_ic is used as an IC counter
        //a.ii
		for(uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++)	// This loop parses GPIO voltages stored in the register
        {								   													 
          aux_codes[current_ic][current_gpio +((gpio_reg-1)*GPIO_IN_REG)] = data[data_counter] + (data[data_counter+1]<<8);
          data_counter=data_counter+2;
        }
		//a.iii
        received_pec = (data[data_counter]<<8)+ data[data_counter+1];
        data_pec = LTC6804_2::pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT*(gpio_reg-1)]);
        if(received_pec != data_pec)
        {
          pec_error = -1;
        }
        data_counter=data_counter+2;
      }
    }
  }
  else
  {
	//b.i
    this->rdaux_reg(reg, data);
    for (int current_ic = 0 ; current_ic < total_ic; current_ic++) // executes for every LTC6804 in the stack
    {							   // current_ic is used as an IC counter
		//b.ii
		for(int current_gpio = 0; current_gpio<GPIO_IN_REG; current_gpio++)   // This loop parses the read back data. Loops 
		{						   // once for each aux voltage in the register 
			aux_codes[current_ic][current_gpio +((reg-1)*GPIO_IN_REG)] = 0x0000FFFF & (data[data_counter] + (data[data_counter+1]<<8));
			data_counter=data_counter+2;
		}
		//b.iii
		received_pec = (data[data_counter]<<8) + data[data_counter+1];
        data_pec = LTC6804_2::pec15_calc(6, &data[current_ic*8*(reg-1)]);
        if(received_pec != data_pec)
        {
          pec_error = -1;
        }
	}
  }
  free(data);
  return (pec_error);
}
/*LTC6804_rdaux Sequence
	
	1. Switch Statement:
		a. Reg = 0
			i. Read GPIO voltage registers A-D for every IC in the stack
			ii. Parse raw GPIO voltage data in cell_codes array
			iii. Check the PEC of the data read back vs the calculated PEC for each read register command
		b. Reg != 0 
			i.Read single GPIO voltage register for all ICs in stack
			ii. Parse raw GPIO voltage data in cell_codes array
			iii. Check the PEC of the data read back vs the calculated PEC for each read register command
	2. Return pec_error flag*/


/*Read the raw data from the LTC6804 auxiliary register
 
 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of 
 the LTC6804_rdaux() command. 
 
 @param[in] uint8_t reg; This controls which GPIO voltage register is read back.   
          1: Read back auxiliary group A
          2: Read back auxiliary group B 
         
 @param[in] uint8_t total_ic; This is the number of ICs in the stack
 
 @param[out] uint8_t *data; An array of the unparsed aux codes*/
void LTC6804_2::rdaux_reg(uint8_t reg, uint8_t *data)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  
  //1
  if (reg == 1)
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }
  else if(reg == 2)
  {
    cmd[1] = 0x0e;
    cmd[0] = 0x00;
  } 
  else
  {
     cmd[1] = 0x0C;
     cmd[0] = 0x00;
  }
  //2
  cmd_pec = LTC6804_2::pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
  //4
   for(int current_ic = 0; current_ic<total_ic; current_ic++)
  {
	cmd[0] = 0x80 + (current_ic<<3); //Setting address
    cmd_pec = LTC6804_2::pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec); 
	output_low(this->spi->cs);
	this->spi_write_read(cmd,4,&data[current_ic*8],8);
	output_high(this->spi->cs);
  }
}
/*
  LTC6804_rdaux_reg Function Process:
  1. Determine Command and initialize command array
  2. Calculate Command PEC
  3. Wake up isoSPI, this step is optional
  4. Send Global Command to LTC6804 stack
*/

/********************************************************//**
 \brief Clears the LTC6804 cell voltage registers
 
 The command clears the cell voltage registers and intiallizes 
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
************************************************************/
void LTC6804_2::clrcell()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  
  //1
  cmd[0] = 0x07;
  cmd[1] = 0x11;
  
  //2
  cmd_pec = LTC6804_2::pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec );
  
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  
  //4
  output_low(this->spi->cs);
  this->spi_write_read(cmd,4,0,0);
  output_high(this->spi->cs);
}
/*
  LTC6804_clrcell Function sequence:
  
  1. Load clrcell command into cmd array
  2. Calculate clrcell cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast clrcell command to LTC6804 stack
*/


/***********************************************************//**
 \brief Clears the LTC6804 Auxiliary registers
 
 The command clears the Auxiliary registers and intiallizes 
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
***************************************************************/
void LTC6804_2::clraux()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  
  //1
  cmd[0] = 0x07;
  cmd[1] = 0x12;
  
  //2
  cmd_pec = LTC6804_2::pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  //4
  output_low(this->spi->cs);
  this->spi_write_read(cmd,4,0,0);
  output_high(this->spi->cs);
}
/*
  LTC6804_clraux Function sequence:
  
  1. Load clraux command into cmd array
  2. Calculate clraux cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast clraux command to LTC6804 stack
*/


/*****************************************************//**
 \brief Write the LTC6804 configuration register 
 
 This command will write the configuration registers of the stacks 
 connected in a stack stack. The configuration is written in descending 
 order so the last device's configuration is written first.
 

@param[in] uint8_t total_ic; The number of ICs being written. 
 
@param[in] uint8_t *config an array of the configuration data that will be written, the array should contain the 6 bytes for each
 IC in the stack. The lowest IC in the stack should be the first 6 byte block in the array. The array should
 have the following format:
 |  config[0]| config[1] |  config[2]|  config[3]|  config[4]|  config[5]| config[6] |  config[7]|  config[8]|  .....    |
 |-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 |IC1 CFGR0  |IC1 CFGR1  |IC1 CFGR2  |IC1 CFGR3  |IC1 CFGR4  |IC1 CFGR5  |IC2 CFGR0  |IC2 CFGR1  | IC2 CFGR2 |  .....    |
 
 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a stack.
********************************************************/
void LTC6804_2::wrcfg(uint8_t config[][6])
{
  const uint8_t BYTES_IN_REG = 6;//it is 6 because tx_cfg[][] has 6 cells
  const uint8_t CMD_LEN = 4+(8*total_ic);
  uint8_t *cmd;
  uint16_t temp_pec;
  uint8_t cmd_index; //command counter
  
  cmd = (uint8_t *) malloc(CMD_LEN*sizeof(uint8_t));
  //1
  cmd[0] = 0x00;//a1)first 8 bits of WRCFG command
  cmd[1] = 0x01;//a2)rest of 8 bits of WRCFG command
  cmd[2] = 0x3d;//b1)first 8 bits of PEC
  cmd[3] = 0x6e;//b2 rest of 8 bits of PEC
  
  //2
  cmd_index = 4;
  for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--) 			// executes for each LTC6804 in stack, 
  {								
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each byte in the CFGR register
    {							// i is the byte counter
	
      cmd[cmd_index] = config[current_ic-1][current_byte]; 		//adding the config data to the array to be sent 
      cmd_index = cmd_index + 1;                
    }
	//3
    temp_pec = (uint16_t) LTC6804_2::pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);// calculating the PEC for each board
    cmd[cmd_index] = (uint8_t)(temp_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)temp_pec;
    cmd_index = cmd_index + 2;
  }
  
  //4
  wakeup_idle(); 															 //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  //5
   for(int current_ic = 0; current_ic<total_ic; current_ic++)
  {
	cmd[0] = 0x80 + (current_ic<<3); //Setting address //0x80 --> B10000000
    temp_pec = LTC6804_2::pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(temp_pec >> 8);
	cmd[3] = (uint8_t)(temp_pec); 
	output_low(this->spi->cs);
	this->spi_write_array(4,cmd);
	this->spi_write_array(8,&cmd[4+(8*current_ic)]);
	output_high(this->spi->cs);
  }
  free(cmd);

  delay(delay_on_send_ms);
}
/*
	1. Load cmd array with the write configuration command and PEC
	2. Load the cmd with LTC6804 configuration data
	3. Calculate the pec for the LTC6804 configuration data being transmitted
	4. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
	5. Write configuration of each LTC6804 on the stack

*/

/* Reads configuration registers of a LTC6804 stack

@param[in] uint8_t total_ic: number of ICs in the stack

@param[out] uint8_t *r_config: array that the function will write configuration data to. The configuration data for each IC 
is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes 
of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_config[0]|r_config[1]|r_config[2]|r_config[3]|r_config[4]|r_config[5]|r_config[6]  |r_config[7] |r_config[8]|r_config[9]|  .....    |
|-----------|-----------|-----------|-----------|-----------|-----------|-------------|------------|-----------|-----------|-----------|
|IC1 CFGR0  |IC1 CFGR1  |IC1 CFGR2  |IC1 CFGR3  |IC1 CFGR4  |IC1 CFGR5  |IC1 PEC High |IC1 PEC Low |IC2 CFGR0  |IC2 CFGR1  |  .....    |

@return int8_t PEC Status.
	0: Data read back has matching PEC
	-1: Data read back has incorrect PEC */
int8_t LTC6804_2::rdcfg(uint8_t r_config[][8])
{
  const uint8_t BYTES_IN_REG = 8;
  
  uint8_t cmd[4];
  uint8_t *rx_data;
  int8_t pec_error = 0; 
  uint16_t data_pec;
  uint16_t received_pec;
  rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));
  //1
  cmd[0] = 0x00;
  cmd[1] = 0x02;
  cmd[2] = 0x2b;
  cmd[3] = 0x0A;
 
  //2
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  //3
   for(int current_ic = 0; current_ic<total_ic; current_ic++)
  {
	cmd[0] = 0x80 + (current_ic<<3); //Setting address
    data_pec = LTC6804_2::pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(data_pec >> 8);
	cmd[3] = (uint8_t)(data_pec); 
	output_low(this->spi->cs);
	this->spi_write_read(cmd,4,&rx_data[current_ic*8],8);
	output_high(this->spi->cs);
  }
 
  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) //executes for each LTC6804 in the stack
  { 
    //4.a
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      r_config[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
    }
    //4.b
    received_pec = (r_config[current_ic][6]<<8) + r_config[current_ic][7];
    data_pec = LTC6804_2::pec15_calc(6, &r_config[current_ic][0]);
    if(received_pec != data_pec)
    {
      pec_error = -1;
    }  
  }
  free(rx_data);
  //5
  return(pec_error);
}
/*1. Load cmd array with the write configuration command and PEC
	2. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
	3. read configuration of each LTC6804 on the stack
	4. For each LTC6804 in the stack
	  a. load configuration data into r_config array
	  b. calculate PEC of received data and compare against calculated PEC
	5. Return PEC Error */

void LTC6804_2::wakeup_idle()
{
  output_low(this->spi->cs);
  delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
  output_high(this->spi->cs);
}
void LTC6804_2::wakeup_sleep()
{
  output_low(this->spi->cs);
  delay(1); //Guarantees the LTC6804 will be in standby
  output_high(this->spi->cs);
}


//brief calaculates  and returns the CRC15
uint16_t LTC6804_2::pec15_calc(uint8_t len, uint8_t * data)
{
	uint16_t remainder = 16,addr;
	for(uint8_t i = 0; i < len; i++)
	{
    //calculate PEC table address 
		addr = ((remainder >> 7) ^ data[i]) & 0xff;
		remainder = (remainder << 8) ^ crc15Table[addr];
	}
  //The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
	return remainder * 2;
}


//brief Writes an array of bytes out of the SPI port
void LTC6804_2::spi_write_array(uint8_t len, uint8_t data[])
{ for(uint8_t i = 0; i < len; i++){ this->spi->write((char)data[i]); }}

//Writes and read a set number of bytes using the SPI port.
void LTC6804_2::spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len)
{
  for(uint8_t i = 0; i < tx_len; i++){ this->spi->write(tx_Data[i]);  }

  for(uint8_t i = 0; i < rx_len; i++){  rx_data[i] = (uint8_t) this->spi->read(0xFF); }
}





