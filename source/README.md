# ASN(x) C Code Library #

The *ASN(x) C Code Library* is a collection of (mainly) self-written ANSI-C libraries and test applications for the *ASN(x)* sensor node.
It offers basic functionality and helper functions for a range of tasks needed to develop a wireless sensor node based on the *ASN(x)*.
The build system is makefile-based and requires only a minimum number of other libraries and tools.


## Prerequisites ##

To use the *ASN(x) C Code Library*, you need to have `avr-gcc`, `AVR Libc`, and `AVRDUDE` installed:  
  `sudo apt install gcc-avr avr-libc avrdude`  
That's basically it :)

Additionally, to generate the doxygen-based documentation, `doxygen` and `graphviz` are required where the latter one is needed to generate the call-graphs:  
  `sudo apt install doxygen graphviz`


## Directory Structure ##

```
.
├── _asnx_lib_                         : ASN(x) library files
├── makefile.in                        : Superior makefile to be included
├── 000-blinky_demo                    : Blinky demo to check the toolchain
├── 001-adc_uart_demo                  : ADC to UART output demo
├── 002-sensor_demo                    : Sensor reading demo
├── 003-xbee_demo                      : XBee basic communication demo
├── 004-sensor_node_demo               : Full sensor node demo
├── xyz                                : Further applications
└── ...
```

## AVRDUDE Basic Usage ##

Flashing the MCU and setting its fuses is done with [AVRDUDE](https://www.nongnu.org/avrdude/).
The basic commands are:
* **Flashing**:  
  `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U flash:w:BINARY.hex`  
  The same can be achieved via the makefile by executing:  
  `make flash`
* **Erasing**:  
  `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -e`  
  The same can be achieved via the makefile by executing:  
  `make erase`
* **Write fuses**:
    * *Low fuses*:  
      `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U lfuse:w:0xXX:m`
    * *High fuses*:  
      `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U hfuse:w:0xXX:m`
    * *Extended fuses*:  
      `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U efuse:w:0xXX:m`
    * The default fuse setings for the ASN(x) are:  
      `avrdude -p atmega1284p -c avrispv2 -P /dev/ttyACM0 -v -U lfuse:w:0xED:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m`  
      The same can be achieved via the makefile by executing:  
      `make defaultfuses`


## XBee 3 Zigbee ##

The XBee 3 is a powerful radio offering a wide range of functions.
In addition to Zigbee, 802.15.4 and Digimesh support (selectable by writing the appropriate firmware), it also has a BLE interface (disabled by default).
Using the BLE interface, you can configure the module using a smartphone in combination with the _Digi XBee Mobile_ app.
However, if you use the XBee for the first time and/or want to configure fresh modules, it is advisable to use Digi's [XCTU](https://www.digi.com/products/embedded-systems/digi-xbee/digi-xbee-tools/xctu) tool on a PC (Windows, Linux, Mac).
For that purpose, you will need to use a XBee to serial adapter such as the [Wavexhare XBee USB Adapter](https://www.waveshare.com/wiki/XBee_USB_Adapter).
Alternatively, the configuration can also be done via the UART interface and AT commands.

There is a [list of libraries](https://www.digi.com/resources/documentation/Digidocs/90001456-13/concepts/c_xbee_libraries_api_mode.htm?TocPath=XBee%20API%20mode%7C_____6) available to interface with the XBee modules.
In addition, you can simply use the module in transparent mode (tunnel UART) or program your own API function (e.g., as shown for [ATmega32](https://www.electronicwings.com/avr-atmega/xbee-interfacing-with-atmega32)).


### Configuration for ASN(x) ###

In the following, the basic configuration of the XBee 3 modules for the use with our ASN(x) sensor nodes is described.
Depending on your application, requirements, etc. you may need to adapt certain parameters.
The explanation of the particular parameters and example configurations can be found in the [XBee 3 User Manual](https://www.digi.com/resources/documentation/digidocs/pdfs/90001539.pdf).
If you use the [XCTU](https://www.digi.com/products/embedded-systems/digi-xbee/digi-xbee-tools/xctu) tool for the configuration, the parameters are grouped to categories; the same as we use below.
Parameters not mentioned in the list below are left at their initial/default value.

* **Networking**
    * _CE_ (Device Role)  
        set to `0` (join network).
    * _ID_ (extended PAN ID)  
        set to a defined address, e.g., `FEDCBA9876543210`.

* **Discovery Options**
    * _NI_ (node identifier)  
        you can set a user-defined name; we use `SNx` for our devices where `x` is a consecutive number.

* **RF Interfacing**
    * _PL_ (TX power level)  
        to save energy, we use the lowest value `0` (-5 dBm).

* **Sleep Settings**
    * _SM_ (sleep mode)  
        to enable the sleep mode, set this value to `1` (pin hibernate).  
        Be careful to have the sleep-request pin at the right logic level; otherwise the XBee goes to sleep and will not be accessible anymore!

* **Bluetooth Options**
    * We use the BLE interface during development to have an easy access to the XBee's setting via the [Digi XBee mobile app](https://www.digi.com/products/embedded-systems/digi-xbee/digi-xbee-tools/digi-xbee-mobile-app).  
        However, in the actual deployment, the BLE interface is deactivated to save energy and for security reasons.
    * _BT_ (Bluetooth enable)  
        activate the BLE interface by setting this parameter to `1` (you will be asked to set a BLE password).
    * _BI_ (Bluetooth identifier)  
        optionally, a user-defined BLE identifier can be set to easily identify the single XBees in the mobile app.  
        We use `XBee SNx` where the `SNx` part matches the _NI_ described above.  
        The prefix `XBee` is used to filter the available Bluetooth devices in the mobile app (i.e., only show Bluetooth devices whose identifier starts with `XBee`).
    * _BP_ (Bluetooth power)  
        to save energy, we use the lowest value `0` (-20 dBm).

* **API Configuration**
    * _AP_ (API enable)  
        we use the XBee in API mode, therefore, set this parameter to `1` (API mode without escapes).

* **UART Interface**
    * We use the default setting of the UART interface (i.e., 9600 8N1).
    * _BD_ (UART baud rate)  
        set to `3` (9600 b/s)
    * _NB_ (parity)  
        set to `0` (no parity)
    * _SB_ (stop bits)  
        set to `0` (one stop bit)

* **I/O Settings**
    * To save energy, we deactivate the GPIOs usually used for status information.  
        However, be careful to leave the sleep request and asleep indicator assigned, especially when using pin hibernation!
    * _D5_ (DIO5/Associate Configuration)  
        set to `0` (disabled)
    * _D8_ (DIO8/Sleep_Rq)  
        set to `1` (DTR/Sleep_Rq)
    * _D9_ (DIO9/Sleep Indicator)  
        set to `1` (Awake/Asleep indicator)
    * _P0_ (DIO10/RSSI Configuration)  
        set to `0` (disabled)


### Resources ###

**XBee firmwares**:
* [Zigbee](https://www.digi.com/products/embedded-systems/digi-xbee/rf-modules/2-4-ghz-rf-modules/xbee3-zigbee-3)
* [802.15.4](https://www.digi.com/products/embedded-systems/digi-xbee/rf-modules/2-4-ghz-rf-modules/xbee3-802-15-4)
* [Digimesh](https://www.digi.com/products/embedded-systems/digi-xbee/rf-modules/2-4-ghz-rf-modules/xbee3-digimesh-2-4)

**XBee libraries**:
* [XBee mbed Library](https://os.mbed.com/teams/Digi-International-Inc/code/XBeeLib/)
* [Digi XBee Ansi C Library](https://github.com/digidotcom/xbee_ansic_library/)
* [XBee-arduino](https://github.com/andrewrapp/xbee-arduino)

If you face troubles with the XBee module, have a look at the [common XBee mistakes](https://www.faludi.com/projects/common-xbee-mistakes/) summary.
