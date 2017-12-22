/*
LT_SPI: Routines to communicate with Teensy's hardware SPI port.

LT_SPI implements the low level master SPI bus routines using
the hardware SPI port. 
*/

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "LT_SPI.h"

LT_SPI::LT_SPI(uint8_t sck,
                uint8_t mosi,
                uint8_t miso,
                uint8_t cs,
                uint8_t spi_clock_divider,
                uint8_t spi_mode) : cs(cs)
{
  //Setup the processor for hardware SPI communication.
  pinMode(sck, OUTPUT);
  pinMode(mosi, OUTPUT);
  pinMode(miso, INPUT);
  pinMode(cs, OUTPUT);
  
  SPI.begin();
  SPI.setClockDivider(spi_clock_divider);
  SPI.setDataMode(spi_mode);
}

LT_SPI::~LT_SPI()
{
  //Disable the SPI hardware port
  SPI.end();
}


void LT_SPI::write(int8_t  data)
{
  //Write a data byte using the SPI hardware
  SPDR = data;           
  //Wait until transfer complete       
  while (!(SPSR & _BV(SPIF)));
}


int8_t LT_SPI::read(int8_t  data) //The data byte to be written
{
  //Read and write a data byte using the SPI hardware
  SPDR = data;                  //Start the SPI transfer
  while (!(SPSR & _BV(SPIF)));  //Wait until transfer complete
  return SPDR;                  //Return the data read
}
