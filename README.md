# ILI948x_t41_p
## A basic display driver for ILI948X series on a Teensy 4.1

**Disclaimer: This is an experimental library, currently a WIP. I cannot guarantee that all functions will work nor can I guarantee that this library will work with other libraries. Use at your own risk**  

This library can communicate with an ILI9488 TFT LCD via an 8 bit parallel interface (8080)
It utilizes FlexIO  to write data to the screen while. 
The basic method is a blocking tranfser eg the application has to wait for the image to be transferred in order to continue running
There is a semi-async method as well that can transfer 32 bytes of data at a time using the FlexIO interrupt feature. While this is not fully async, it can ease off some load from the MCU while transferring the image.

It can only write an image array at the moment with defined start/end coordinates.
The default bus speed is set to 12Mhz and can be lowered or raised with a simple function call (see below)

First include the library and create a constructor:
```
#include "ILI948x_t1_p.h"
#define CS 11
#define DC 13
#define RST 12
ILI948x_t41_p lcd = ILI948x_t41_p(DC,CS,RST);
```
You can use any GPIO pins for CS, DC and RST

Next, wire up your LCD - use Teensy pins:
* D0 - pin 19
* D1 - pin 18
* D2 - pin 14
* D3 - pin 15
* D4 - pin 40
* D5 - pin 41
* D6 - pin 17
* D7 - pin 16

* WR - pin 36
* RD - pin 37 or 3.3v if no read command is needed
   


in the setup function call:
```
ILI948x_t41_p::begin();
```
The default baud rate is 20Mhz

In the begin(n) function you can pass 2,4,8,12,20,24, 30 and 40 to lower or raise the baud rate.


Call the following function for a polling method write:
```
ILI948x_t41_p::pushPixels16bit(flexio_teensy_mm,0,0,480,320);
```
or call the following function for an async interrupt based write (this is not fully async, but will output 32 bytes between interrupts)
```
ILI948x_t41_p::pushPixels16bitAsync(flexio_teensy_mm,0,0,480,320);
```
to push the image data, the arguments are as follows:
* uint16_t color array (RGB565)
* uint16_t x1
* uint16_t y1
* uint16_t x2
* uint16_t y2

Additional API's:


Set rotation: 1,2,3,4
```
ILI948x_t41_p::setRotation(n);
```

Invert display color (true/false)
```
ILI948x_t41_p::invertDisplay(bool);
```

Register a callback to trigger when the Async transfer completes.
```
ILI948x_t41_p::onCompleteCB(CBF callback);
```
![Image of TFT with Teensy MM image](https://github.com/david-res/ILI948x_t41_p/blob/main/mm_flexio_example.jpg)

