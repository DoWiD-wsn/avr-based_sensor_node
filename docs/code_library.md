# ASN(x) C Code Library #

The ASN(x) comes with a software library written in ANSI-C language, called `ASNX LIB`, to lower the entry barrier and ease the development of the node's software.
It provides a set of modules for the interaction with the AVR MCU's on-chip peripherals (e.g., ADC, UART, timer), several external sensor libraries connected via I²C or a one-wire interface (OWI), a high-level library to communicate with the Xbee 3 radio transceiver, and high level modules to support unit conversions as well as packing sensor values into messages (including a fixed-point number representation).

The libraries where self-written (unless otherwise specified) and have only a small number of dependencies.
That is, they only require `AVR Libc` to be available on the system and a recent version of `avr-gcc` for the compilation.
To flash the application on the MCU, either the tools provided by [https://www.microchip.com/en-us/development-tools-tools-and-software](Atmel) or `AVRDUDE` can be used.
For more information on the installation of those and the basic usage of `AVRDUDE` refer to the source directory's [../source/README.md](README).

To further ease the development of the ASN(x) software part, we provide a superior makefile that offers basic functionality in a convenient way, namely
* `flash` ... the flashing of a binary onto the MCU
* `clean` ... the cleaning of intermediate files
* `erase` ... the erasing of the MCU (clear the flash memory)
* `defaultfuses` ... the programming of the default fuse settings (lb: `0xDD`, hb: `0xD9`, eb: `0xff`)
* `doc` ... the generation of the libraries doxygen documentation
* `distclean` ... the cleaning of everything, that is, intermediate files and documentary

The generation of the documentation requires doxygen and dotviz (to generate the call-graphs), see [../source/README.md](README).


## ASNX LIB dependencies ##

As mentioned above, the ASN(x) software library has only little external dependencies (i.e., `AVR Libc`).
Additionally, the internal structure was kept shallow to minimize the modules' cross-dependencies and keep the software complexity as low as reasonably possible.

Dependency graph:
![PCB front (/media/asnx_lib/dependency_graph.svg)](../media/asnx_lib/dependency_graph.svg)


## Prepared demo applications ##

In the source directory, you can also find five pre-prepared demo applications.
They show the usage of the `ASNX LIB` and the ASN(x) components in a bottom-up fashion.

### 000-blinky_demo ###

The `blinky demo` is intended to help getting familiar with the toolchain and getting the hardware up and running (i.e., compilation and flashing of the software)
If everything worked, the two user LEDs will blink alternately with a delay of 500ms.
This demo only uses the `LED` module.

### 001-adc_uart_demo ###

The second demo application additionally shows the usage of the `ADC` and the `UART` modules as well as a simple `printf` library.
In the demo, the MCU's supply voltage and the battery voltage are read once per second (via the `ADC`) and the results are printed using the `printf` library on the `UART1` interface.

### 002-sensor_demo ###

The `sensor demo` depicts the usage of the provided external sensor libraries and the timer-based systick timer.
The following sensors are currently supported:
* DS18B20 temperature sensor (OWI)
* AM2302 temperature/humidity sensor (DHT22; OWI)
* BME280 environmental sensor (TWI)
* TMP275 on-board temperature sensor (TWI)
* LM75 temperature sensor (similar to TMP275; TWI)
* STEMMA soil humidity sensor (TWI)

Additionally, the MCU surface temperature measurement via the on-board voltage divider and an external 103JT thermistor (10kOhm @25°C) based on the Steinhart–Hart equation is supported.
All sensor values are read and printed via UART in a definable interval controlled by the systick timer.


### 003-xbee_demo ###

The fourth demo application shows the interaction with the Xbee 3 radio module connected via UART0.
It sends a steadily increasing number via unicast to a specified network participant (specified via its 64-bit MAC address) every 2 seconds and checks the transmission response (acknowledgment).


### 004-sensor_node_demo ###

The last demo, the full `sensor node demo`, shows an example of a full sensor node including Xbee communication, sensor readings, and sleep modes.
In this demo also the on-board RTC is used as a wake-up source for the MCU when it is sleeping.
Basically, the sensor node is woken up every 10 minutes (interval can be configured), reads the current sensor values, sends them to a defined network participant and goes back to sleep (power-down mode).
In addition to the basic sensor readings, also some diagnostic data are sent.


## Development of an own application ##

To create an own application, we suggest the following procedure:
* Create a new directory in the source folder.
* Create a C source file in the newly created directory (e.g., `APPLICATION_NAME.c`)
* Put a makefile into the new directory that contains at least:
** `TARGET = APPLICATION_NAME`
** `include ../makefile.in`
** Additionally, also the port or other specific settings can be configured in the makefile.

That's it.
You are ready to code your own application.

