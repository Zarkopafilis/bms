/*
LT_SPI: Routines to communicate with Teensy's hardware SPI port.

LT_SPI implements the low level master SPI bus routines using
the hardware SPI port. 
*/

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "LT_SPI.h"

//Default Slave Select pin - used as an "adapter" define
#define QUIKEVAL_CS SS

// Reads and sends a byte
// Return 0 if successful, 1 if failed
void spi_transfer_byte(uint8_t cs_pin, uint8_t tx, uint8_t *rx)
{
  output_low(cs_pin);                 //! 1) Pull CS low

  *rx = SPI.transfer(tx);             //! 2) Read byte and send byte

  output_high(cs_pin);                //! 3) Pull CS high
}

// Reads and sends a word
// Return 0 if successful, 1 if failed
void spi_transfer_word(uint8_t cs_pin, uint16_t tx, uint16_t *rx)
{
  union
  {
    uint8_t b[2];
    uint16_t w;
  } data_tx;

  union
  {
    uint8_t b[2];
    uint16_t w;
  } data_rx;

  data_tx.w = tx;

  output_low(cs_pin);                         //! 1) Pull CS low

  data_rx.b[1] = SPI.transfer(data_tx.b[1]);  //! 2) Read MSB and send MSB
  data_rx.b[0] = SPI.transfer(data_tx.b[0]);  //! 3) Read LSB and send LSB

  *rx = data_rx.w;

  output_high(cs_pin);                        //! 4) Pull CS high
}

// Reads and sends a byte array
void spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx, uint8_t length)
{
  int8_t i;

  output_low(cs_pin);                 //! 1) Pull CS low

  for (i=(length-1);  i >= 0; i--)
    rx[i] = SPI.transfer(tx[i]);    //! 2) Read and send byte array

  output_high(cs_pin);                //! 3) Pull CS high
}

// Setup the processor for hardware SPI communication.
// Must be called before using the other SPI routines.
void spi_enable(uint8_t spi_clock_divider) // Configures SCK frequency. Use constant defined in header file.
{
  pinMode(SCK, OUTPUT);             //! 1) Setup SCK as output
  pinMode(MOSI, OUTPUT);            //! 2) Setup MOSI as output
  pinMode(MISO, INPUT);
  pinMode(QUIKEVAL_CS, OUTPUT);     //! 3) Setup CS as output
  SPI.begin();
  SPI.setClockDivider(spi_clock_divider);
  //SPI.setDataMode(SPI_MODE3);
}

//Disable the SPI hardware port
void spi_disable()
{
  SPI.end();
}

// Write a data byte using the SPI hardware
void spi_write(int8_t  data)  // Byte to be written to SPI port
{
  SPDR = data;                  //! 1) Start the SPI transfer
  while (!(SPSR & _BV(SPIF)));  //! 2) Wait until transfer complete
}

// Read and write a data byte using the SPI hardware
// Returns the data byte read
int8_t spi_read(int8_t  data) //!The data byte to be written
{
  SPDR = data;                  //! 1) Start the SPI transfer
  while (!(SPSR & _BV(SPIF)));  //! 2) Wait until transfer complete
  return SPDR;                  //! 3) Return the data read
}

void output_low(uint8_t pin){
  digitalWrite(pin, LOW);
}

void output_high(uint8_t pin){
  digitalWrite(pin, HIGH);
}
