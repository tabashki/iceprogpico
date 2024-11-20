Raspberry Pi Pico iCE40 Programmer
==================================

Flash programmer for the OLIMEX iCE40HX8K-EVB FPGA development board,
based on the [iceprogduino](https://github.com/OLIMEX/iCE40HX1K-EVB/tree/master/programmer/iceprogduino)
project and accompanying [firmware](https://github.com/OLIMEX/iCE40HX1K-EVB/tree/master/programmer/olimexino-32u4%20firmware).


Build Instructions
==================

- Ensure you have the latest version of [pico-sdk](https://github.com/raspberrypi/pico-sdk.git) and ensure that `PICO_SDK_PATH` is correctly set
- Generate build files using: `mkdir build && cd build && cmake .. --build`
- You should now have a `iceprogpico.uf2` file ready for USB flashing