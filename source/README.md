# ASN(x) C Code Library #

The *ASN(x) C Code Library* is a collection of (mainly) self-written ANSI-C libraries and test applications for the *ASN(x)* sensor node.
It offers basic functionality and helper functions for a range of tasks needed to develop a wireless sensor node based on the *ASN(x)*.
The build system is makefile-based and requires only a minimum number of other libraries and tools.


## Prerequisites ##

To use the *ASN(x) C Code Library*, you need to have `avr-gcc`, `AVR Libc`, and `AVRDUDE` installed:  
  `sudo apt install gcc-avr avr-libc avrdude`  
That's basically it :)


## Directory Structure ##

```
.
├── _common_                           : Library files and other common stuff
├── 000-blinky_demo                    : Initial blinky demo to check the toolchain
├── 001-adc_uart_demo                  : ADC to UART output demo
├── xyz                                : Further applications
└── ...
```

## AVRDUDE Basic Usage ##

Flashing the MCU and setting its fuses is done with [AVRDUDE](https://www.nongnu.org/avrdude/).
The basic commands are:
* **Flashing**:  
  `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U flash:w:BINARY.hex`
* **Erasing**:  
  `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -e`
* **Write fuses**:
    * *Low fuses*:  
      `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U lfuse:w:0xXX:m`
    * *High fuses*:  
      `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U hfuse:w:0xXX:m`
    * *Extended fuses*:  
      `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U efuse:w:0xXX:m`
* The default fuses for the ASN(x) are:  
  `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U lfuse:w:0xDD:m`  
  `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U hfuse:w:0xD9:m`  
  `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U efuse:w:0xFF:m`


## Xbee3 Zigbee ##

The Xbee3 is a powerful radio offering a wide range of functions.
In addition to Zigbee, 802.15.4 and Digimesh support (selectable by writing the appropriate firmware), it also has a BLE interface (disabled by default).
Using the BLE interface, you can configure the module using a smartphone in combination with the _Digi XBee Mobile_ app.
However, if you use the Xbee for the first time and want to configure fresh modules, it is advisable to use Digi's [XCTU](https://www.digi.com/products/embedded-systems/digi-xbee/digi-xbee-tools/xctu) tool (Windows, Linux, Mac).

There is a [list of libraries](https://www.digi.com/resources/documentation/Digidocs/90001456-13/concepts/c_xbee_libraries_api_mode.htm?TocPath=XBee%20API%20mode%7C_____6) available to interface with the Xbee modules.
In addition, you can simply use the module in transparent mode (tunnel UART) or program your own API function (e.g., as shown for [ATmega32](https://www.electronicwings.com/avr-atmega/xbee-interfacing-with-atmega32)).

**XBee firmwares**:
* [Zigbee](https://www.digi.com/products/embedded-systems/digi-xbee/rf-modules/2-4-ghz-rf-modules/xbee3-zigbee-3)
* [802.15.4](https://www.digi.com/products/embedded-systems/digi-xbee/rf-modules/2-4-ghz-rf-modules/xbee3-802-15-4)
* [Digimesh](https://www.digi.com/products/embedded-systems/digi-xbee/rf-modules/2-4-ghz-rf-modules/xbee3-digimesh-2-4)

**XBee libraries**:
* [XBee mbed Library](https://os.mbed.com/teams/Digi-International-Inc/code/XBeeLib/)
* [Digi XBee Ansi C Library](https://github.com/digidotcom/xbee_ansic_library/)
* [XBee-arduino](https://github.com/andrewrapp/xbee-arduino)

If you face troubles with the Xbee module, have a look at the [common XBee mistakes](https://www.faludi.com/projects/common-xbee-mistakes/) summary.
