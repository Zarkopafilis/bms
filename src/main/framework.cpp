/* Extensions and additions on LT_SPI and more 
importantly LTC60842 functions to make them work for teensy. */

#include <stdint.h>
#include <Arduino.h>
#include "LT_SPI.h"
#include "LTC68042.h"
#include <SPI.h>
#include "framework.h"

/* Performs Open Wire Checks on a LTC6804 stack

@param[in] uint8_t nIC: number of ICs in the stack

@param[out] uint8_t *r_config: array that the function will write configuration data to. The configuration data for each IC 
is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes 
of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_config[0]|r_config[1]|r_config[2]|r_config[3]|r_config[4]|r_config[5]|r_config[6]  |r_config[7] |r_config[8]|r_config[9]|  .....    |
|-----------|-----------|-----------|-----------|-----------|-----------|-------------|------------|-----------|-----------|-----------|
|IC1 CFGR0  |IC1 CFGR1  |IC1 CFGR2  |IC1 CFGR3  |IC1 CFGR4  |IC1 CFGR5  |IC1 PEC High |IC1 PEC Low |IC2 CFGR0  |IC2 CFGR1  |  .....    |

@return int8_t PEC Status.
	0: Data read back has matching PEC
	-1: Data read back has incorrect PEC */
// int8_t LTC6804_adow(uint8_t nIC){
//     return 0;
// }



/*Performs  a check for the VOV and VUV flags on every slave on the network 
	@param[in] uint8_t nIC: number of ICs in the stack
	@param[out] ovuv: array that the function will write received data to.
	For every slave on the first column,
	For every cell on the second column,
	OV => [slave][cell][0]
	UV => [slave][cell][1]
*/
int8_t LTC68042_ovuv(uint8_t nIC, uint8_t ovuv[][12][2]){
	//Status Register Group B
	//STRB2 C 4~1[OV,UV]
	//STRB3 C 8~5[OV,UV]
	//STRB4 C 12~9[OV,UV]

	return 0;
}