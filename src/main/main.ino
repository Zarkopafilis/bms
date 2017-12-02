#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include "LT_SPI.h"

/* Initializes the connection between the teensy and the attached LTC6804 */
void LTC6804_initialize()
{
  //Clock Driver Constant
  spi_enable(SPI_CLOCK_DIV4);

  //Fast ADC Mode, Dishcharge Disabled, Measure All Cells, Measure All GPIOs
  //set_adc(MD_FAST,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);
}

void setup() {
  LTC6804_initialize();
}

void loop() {
  // put your main code here, to run repeatedly:

  teardown();
}


void teardown(){
  
}

