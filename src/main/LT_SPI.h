//LT_SPI: Routines to communicate with Teensy's hardware SPI port.


#ifndef LT_SPI_H
#define LT_SPI_H

#include <stdint.h>
#include <SPI.h>

//! Reads and sends a byte
//! @return void
void spi_transfer_byte(uint8_t cs_pin,      //!< Chip select pin
                       uint8_t tx,          //!< Byte to be transmitted
                       uint8_t *rx          //!< Byte to be received
                      );

//! Reads and sends a word
//! @return void
void spi_transfer_word(uint8_t cs_pin,      //!< Chip select pin
                       uint16_t tx,         //!< Byte to be transmitted
                       uint16_t *rx         //!< Byte to be received
                      );

//! Reads and sends a byte array
//! @return void
void spi_transfer_block(uint8_t cs_pin,     //!< Chip select pin
                        uint8_t *tx,        //!< Byte array to be transmitted
                        uint8_t *rx,        //!< Byte array to be received
                        uint8_t length      //!< Length of array
                       );

//! Setup the processor for hardware SPI communication.
//! Must be called before using the other SPI routines.
void spi_enable(uint8_t spi_clock_divider   //!< Configures SCK frequency. Use constant defined in header file.
               );

//! Disable the SPI hardware port
void spi_disable();

//! Write a data byte using the SPI hardware
void spi_write(int8_t data  //!< Byte to be written to SPI port
              );

//! Read and write a data byte using the SPI hardware
//! @return the data byte read
int8_t spi_read(int8_t data //!< The data byte to be written
               );

//Wrapper to redirect calls to use the proper digitalWrite functions
void output_low(uint8_t pin);
void output_high(uint8_t pin);

#endif  // LT_SPI_H
