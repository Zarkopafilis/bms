/*LT_SPI implements the low level master SPI bus routines using
the teensy's  SPI capabilities. */

#ifndef LT_SPI_H
#define LT_SPI_H

#include <stdint.h>
#include <SPI.h>

class LT_SPI{
  public:
    LT_SPI(uint8_t sck = SCK,
                uint8_t mosi = MOSI,
                uint8_t miso = MISO,
                uint8_t cs = SS,
                uint8_t spi_clock_divider = SPI_CLOCK_DIV16,
                uint8_t spi_mode = SPI_MODE3);
    ~LT_SPI();
    
    //Write a data byte
    void write(int8_t data);
    
    //Read and write a data byte
    int8_t read(int8_t data);

    const uint8_t cs;
};

#endif  // LT_SPI_H
