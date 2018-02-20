# AS5048A-C2000
AS5048A SPI library for the TI LaunchXL-F28027F

This library written in C was created using the Texas Instruments Launchpad: LaunchXL-F28027F together with Code Composer Studio (CCS) and uses the controlSUITE libraries. It was made for the LaunchXL-F28027F, but it would work on other board as well. Remember to check the GPIO pin configuration.

To use, add the AS5048A.c and AS5048A.h into your project and include the header file. An example is given where all the variables and registeres are read. Only part that hasnt been tested is the buring of the offset value.

GPIOs used:
MOSI  = GPIO16
MISO  = GPIO17
CLK   = GPIO18
CS    = GPIO19
