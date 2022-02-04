# Known Issues

In the following, known issues with the ASN(x) soft- or hardware are listed and possible fixes are provided.
Currently, the following issues are known:

* [DC/DC Converter Startup](#dcdc-converter-startup)


## DC/DC Converter Startup

The ASN(x) uses a [TPS63031DSKR](docs/datasheets/TPS63031DSKR.pdf) low-power DC/DC converter with a fixed output voltage of 3.3 V.
While providing a stable supply for the sensor node with a comparably high energy efficiency, we found one issue that may be bothersome during sensor node software development.
That is, when a voltage supply is connected to the DC/DC converter's output (e.g., by an AVR ISP programmer with built-in supply) the converter sometimes does not start-up (work) properly, even if the second source is removed.

A solution to this issue is to make sure not to drive the programmers (or whatever is connected) internal supply; most AVR ISP programmer we found allow one to disable the supply voltage.
However, in case you face this issue it normally helps to remove all power sources from the ASN(x) and leave it unpowered for several minutes.
Normally, after that the sensor node can be normally used again.
