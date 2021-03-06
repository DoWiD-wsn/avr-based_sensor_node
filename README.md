# AVR-based Sensor Node with Xbee (ASN(x))

The **A**VR-based **S**ensor **N**ode with **X**bee, short **ASN(x)**, is a low-cost platform for low-power sensor nodes specifically for (environmental) monitoring applications, both indoor and outdoor.
It was designed with a focus on energy efficiency and robustness.

![PCB 3D (/media/pcb/avr_sensor_node-3d.png)](/media/pcb/avr_sensor_node-3d.png)


## Key Facts

* Compact size of 77 x 43.5 mm
* Powered by two AA batteries (but not limited to; see below)
* [ATmega1284P](docs/datasheets/ATmega1284P.pdf) low-power 8-bit MCU (running at 4MHz)
    * 128kB flash memory
    * 16kB SRAM
    * 4kB EEPROM
* Two UART interfaces
    * one for the radio transceiver
    * one for debug purposes or loopback self-tests
* Programmable via [6-pin AVR-ISP](https://www.mikrocontroller.net/wikifiles/9/97/Avr-isp-pinout.png) connector
* On-board [TMP275](docs/datasheets/TMP275.pdf) temperature sensor (I²C)
* On-board [PCF85263ATT](docs/datasheets/PCF85263A.pdf) RTC as MCU wake-up source (I²C; INT2)
* Fixed 3.3V DC/DC converter [TPS63031DSKR](docs/datasheets/TPS63031DSKR.pdf)
    * Input voltage range: 1.8V to 5.5V
    * Energy efficiency (node power-down): >= 65%
    * Energy efficiency (node active): >= 80%
    * Optimized for low-power mode (<100mA)
* Xbee radio socket
    * UART and SPI connected
    * sleep-request and sleep-indicator pins connected
* Two low-current user LEDs (can be disconnected via cut-through solder jumpers)
* Two solder jumpers to enable bridging of UART0 and UART1 (for loopback self-tests)
* Onboard resistor divider for ADC self-test 
* Onboard resistor divider for battery voltage measurement
* Onboard connector for thermistor temperature measurements
* Two one-wire interface (OWI) connectors (incl. pull-up resistors; separate data pins)
* Two two-wire interface (TWI/I²C) connectors (incl. pull-up resistors)
* 2x14-pin expansion headers


## Contents

```
.
├── docs                : documents & project documentation
│   └── datasheets      : datasheets of components used
├── kicad               : KiCad files
├── media               : Miscellaneous media (images, etc.)
│   ├── pcb             : Photos and rendered images of the PCB
│   └── schematic       : SVG images of the schematics
└── source              : C code library and example programs
```

For more information on the PCB (and its design) refer to [docs/pcb_design.md](docs/pcb_design.md).
The C code library, its functionality, and usage as well as example scripts are presented in [docs/code_library.md](docs/code_library.md).


## Built with

* [KiCad EAD 5.1.9](https://kicad.org/) - PCB design
* [avr-gcc 5.4.0](https://gcc.gnu.org/wiki/avr-gcc) - C code library
* [avr-libc 2.0.0](https://www.nongnu.org/avr-libc/) - C code library


## Contributors

* **Dominik Widhalm** - [***DC-RES***](https://informatics.tuwien.ac.at/doctoral/resilient-embedded-systems/) - [*UAS Technikum Wien*](https://embsys.technikum-wien.at/staff/widhalm/)

Contributions of any kind to improve the project are highly welcome.
For coding bugs or minor improvements simply use pull requests.
However, for major changes or general discussions please contact [Dominik Widhalm](mailto:widhalm@technikum-wien.at?subject=ASN(x)%20on%20GitHub).


## Publications

- Dominik Widhalm, Karl M. Goeschka, and Wolfgang Kastner, "*An open-source wireless sensor node platform with active node-level reliability for monitoring applications*", Sensors 21, no. 22: 7613, 2021, DOI: [10.3390/s21227613](https://doi.org/10.3390/s21227613)


## Changelog

A list of prior versions and changes between the updates can be found in the [CHANGELOG.md](CHANGELOG.md) file.
Note that the major and minor number refer to the hardware version of the ASN(x) board while the patch version is used to denote subsequent software versions.


## Known Issues

A list of known issues with the ASN(x) is available in the [ISSUES.md](ISSUES.md) file.
In this file, we keep track of all kinds of problems/bug and possible fixes for them.


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
