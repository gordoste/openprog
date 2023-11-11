# openprog
openprog firmware

This is a fork of the firmware section of Alberto Maccioni's openprog project (http://openprog.altervista.org/). The primary goal is to provide a version that can be built in MPLAB X with the XC8 compiler, as the firmware was targeting the MCC compiler which is no longer supported by Microchip. This has been achieved and therefore I am releasing the code here under GPLv3.

The section of the firmware presenting the device on the USB bus as a HID device may be useful to others, as it occupies significantly less memory than Microchip's MLA driver.

The change history may be helpful to others looking to port code from MCC to XC8.

The firmware targets the PIC18F25K50, but some support code for other chips remains.

## Testing

Testing methodology for this project is to try to test each programming algorithm once. The different algorithms are implemented on the PC side, and the table below refers to the functions in the PC software (op/opgui) which is available at the [Openprog website](http://openprog.altervista.org). The device compatibility list on that website provides a complete list of supported devices.

Where no device is listed, I do not have a device to perform testing.
Where multiple devices are listed, "PASS" indicates all devices passed. If a number appears in brackets after "PASS", then that device passed but I have not tested any others.

Testing conducted on [Openprog All-In-One board](https://github.com/gordoste/openprog_aio) v1.0b:

|Header|Function|Device1|Device2|Device3|Status|
|------|--------|-------|-------|-------|------|
|progAVR.h  |   WriteAT| |
|progAVR.h	|   WriteATmega|	ATmega88	| ATtiny861A	
|progAVR.h	|   WriteAT_HV|	ATtiny85		
|progAVR.h	|   WriteATfuseSlow|			
|progEEPROM.h|	WriteI2C|	24LC32  |   |   | PASS |		
|progEEPROM.h|	Write93Sx|			
|progEEPROM.h|	Write93Cx|	93C46A		
|progEEPROM.h|	Write25xx|	25AA010		
|progEEPROM.h|	WriteOneWireMem|			
|progEEPROM.h|	Write11xx|			
|progP12.h|	Write12F5xx|	12F510	| 16F527	| 10F202 | PASS (1,2) |
|progP12.h|	Write12C5xx|			
|progP16.h|	Write12F6xx|			
|progP16.h|	Write12F61x|	12F752	| 12F609	|   | PASS (ALL) |
|progP16.h|	Write12F62x|			
|progP16.h|	Write16F7x|			
|progP16.h|	Write16F71x|	16F716		
|progP16.h|	Write16F72x|	16LF721		
|progP16.h|	Write16F62x|			
|progP16.h|	Write16F8x|			
|progP16.h|	Write16F81x|			
|progP16.h|	Write16F87x|			
|progP16.h|	Write16F87xA|			
|progP16.h|	Write16F88x|			
|progP16.h|	Write16F1xxx|	12F1572	| 16F18325	|  | PASS (2) |
|progP16.h|	Write16F18xxx|	16F17114		
|progP18.h|	Write18Fx|	18LF25K50   	| 18F2550 | |PASS (1) |
|progP18.h|	Write18FKx|			
|progP24.h|	Write24Fx|	33FJ128GP802	| | |PASS|
|progP24.h|	Write24Ex|	33EP512MC502	| | |PASS|

