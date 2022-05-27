# openprog
openprog firmware

This is a fork of the firmware section of Alberto Maccioni's openprog project (http://openprog.altervista.org/). The primary goal is to provide a version that can be built in MPLAB X with the XC8 compiler, as the firmware was targeting the MCC compiler which is no longer supported by Microchip. This has been achieved and therefore I am releasing the code here under GPLv3.

The section of the firmware presenting the device on the USB bus as a HID device may be useful to others, as it occupies significantly less memory than Microchip's MLA driver.

The change history may be helpful to others looking to port code from MCC to XC8.

The firmware targets the PIC18F25K50, but some support code for other chips remains.

The following chips have been tested:
12F609
12F1572
