#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include "LT_SPI.h"
#include "LTC68042.h"

void setup() {
  LTC6804_initialize();
}

void loop() {
  // put your main code here, to run repeatedly:

  teardown();
}


void teardown(){
  
}

