Firmware for Open Programmer project,
an USB programmer for PIC and ATMEL micros, I2C memories, and other I2C/SPI devices.
Website: http://openprog.altervista.org/
This firmware is based on the USB firmware released by Alexander Enzmann.
All functions are in OPcontrol.c, which is released under GPL2 license.

Changelog:

V 0.11.2 September 2020:extended ICSP8_READ payload to 16 bits (for PIC18) 
V 0.11.0 January 2019:	added instructions ICSP8_SHORT,ICSP8_READ,ICSP8_LOAD
V 0.10.0 June 2016:  	added instructions LOAD_PC, LOAD_DATA_INC, READ_DATA_INC, JTAG_SET_MODE, JTAG_SEND_CMD, JTAG_XFER_DATA, JTAG_XFER_F_DATA
						new USB VID&PID (0x1209,0x5432), changed some CK timing, reduced CLOCK_GEN startup time
V 0.9.0	 January 2014: 	added instructions SET_PORT_DIR, AT_HV_RTX, SIX_LONG5; improved DCDC control algorithm.
V 0.8.0	 June 2012: 	added one-wire and UNIO protocols; tx buffer sent automatically when rx is 
						elaborated; fixed PROG_C.
V 0.7.6	 July 2010:  	modified TX16 and RX16 with variable clock period; fix for 12 bit ADC.
V 0.7.0	 January 2010:	added some PIC24 instructions, hardware ID.
V 0.6.1	 August 2009:	switched to software SPI due to some bugs in the hardware peripheral.
V 0.6.0	 June 2009:  	switched to GPL USB framework, added instructions for PIC24 and microwire.
V 0.5.0	 November 2008:	added SPI protocol and some ATMEL specific instructions.
V 0.4.0	 August 2008: 	added I2C protocol.
V 0.3.0	 July 2008:		first public release, supports PIC ICSP protocol (PIC10,12,16,18).
