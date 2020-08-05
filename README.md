# weather-relay

This is the software that reads from the serial port on a Raspberry Pi.  I have it running on a 3B+ but also tested it on a ZeroW.
On APRS this software shows up as wx-relay or folabs-wx-relayXXX, XXX being the version number of the relay software.

This code expects to be talking to this firmware https://github.com/crazypill/tx31u-receiver.  That firmware runs on a Feather M0 + RFM69.  It is designed to pickup signals from a WS-1516 compatible 915Mhz weather sensors (specifically the TX31U+ transmitter from LaCrosse).

![](https://raw.githubusercontent.com/crazypill/weather-relay/master/folabs-wx-relay.jpg)
