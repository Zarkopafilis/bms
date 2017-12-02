# Prototypes
Please do not update, edit or in general perform write operations in these files.

These are the prototypes that Linear provides in order to be able to use the LTC Battery Monitors and the isoSPI solution by using their 'Linduino' (Arduino Fork).

# Porting Process Outline
In order to use these libraries with any other MCU, you should read through them and take several steps in order to port them properly:

- Remove QuikEval related code
- Perform the necessary adapter changes in order to make them work with your own/teensy's/arduino's libraries
    For the LT_SPI:
    - Appropriate output_high and output_low functions
    - Appropriate QUIKEVAL_CS definition
    For the LTC68042:

