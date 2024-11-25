Raspberry Pi Pico iCE40 Programmer
==================================

Flash programmer for the OLIMEX iCE40HX8K-EVB FPGA development board,
based on the [iceprogduino](https://github.com/OLIMEX/iCE40HX1K-EVB/tree/master/programmer/iceprogduino)
project and accompanying [firmware](https://github.com/OLIMEX/iCE40HX1K-EVB/tree/master/programmer/olimexino-32u4%20firmware).

Pin Assignments
===============

| RP2024 Pin | Usage    | Pico Pin # | PGM1 Pin # |
|------------|----------|------------|------------|
| GP0        | CDONE    | 1          | 5          |
| GP1        | RESET    | 2          | 6          |
| GP2        | SPI SCK  | 4          | 9          |
| GP3        | SPI MOSI | 5          | 8          |
| GP4        | SPI MISO | 6          | 7          |
| GP5        | SPI CS   | 7          | 10         |


Build Instructions
==================

- Ensure you have the latest version of [pico-sdk](https://github.com/raspberrypi/pico-sdk.git) and ensure that `PICO_SDK_PATH` is correctly set
- Generate build files using: `mkdir build && cd build && cmake .. --build`
- You should now have a `iceprogpico.uf2` file ready for USB flashing