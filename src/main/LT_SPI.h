/*LT_SPI implements the low level master SPI bus routines using
the teensy's  SPI capabilities. */

#ifndef LT_SPI_H
#define LT_SPI_H

#include <stdint.h>
#include <SPI.h>

//Default Slave Select pin - used as an "adapter" define
#define QUIKEVAL_CS SS

//Reads and sends a byte
void spi_transfer_byte(uint8_t cs_pin, uint8_t tx, uint8_t *rx);

//Reads and sends a word
void spi_transfer_word(uint8_t cs_pin, uint16_t tx, uint16_t *rx);

//Reads and sends a byte array
void spi_transfer_block(uint8_t cs_pin,uint8_t *tx, uint8_t *rx,uint8_t length);

//Setup the processor for SPI communication.
//Must be called before using the other SPI routines.
void spi_enable(uint8_t spi_clock_divider);

//Disable the SPI communications
void spi_disable();

//Write a data 
void spi_write(int8_t data);

//Read and write a data byte
//@return the read byte
int8_t spi_read(int8_t data);

//Wrapper to redirect calls to use the proper digitalWrite functions for teensy
void output_low(uint8_t pin);
void output_high(uint8_t pin);

#endif  // LT_SPI_H
