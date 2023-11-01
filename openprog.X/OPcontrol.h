#define _XTAL_FREQ 48000000

#include <stdio.h>

//#define USB_DEBUG
//#define DEBUG

#ifdef DEBUG
#define _DEBUG0(x) { puts(x); putchar('\r'); }
#define _DEBUG1(x,a) { printf(x,a);}
#define _DEBUG2(x,a,b) { printf(x,a,b);}
#define _DEBUG3(x,a,b,c) { printf(x,a,b,c);}
#define _DEBUG4(x,a,b,c,d) { printf(x,a,b,c,d);}
#define _DEBUGS(x) { puts(x); }
#define _DEBUGCH(x) { putchar(x); putchar(' '); }
#else
#define _DEBUG0(x) {}
#define _DEBUG1(x,a) {}
#define _DEBUG2(x,a,b) {}
#define _DEBUG3(x,a,b,c) {}
#define _DEBUG4(x,a,b,c,d) {}
#define _DEBUGS(x) {}
#define _DEBUGCH(x) {}
#endif

#ifdef USB_DEBUG
#define _USBDEBUG0(x) _DEBUG0(x)
#define _USBDEBUG1(x,a) _DEBUG1(x,a)
#define _USBDEBUGCH(x) _DEBUGCH(x)
#else
#define _USBDEBUG0(x)
#define _USBDEBUG1(x,a)
#define _USBDEBUGCH(x)
#endif

#define VERSION "0.12.0"
#define VER2	0
#define VER1	12
#define VER0	0
#define ID2	0
#define ID1	0
#define TX_TEMP_MAX 10
#define LOBYTE(x) (*((byte *)&x))
#define HIBYTE(x) (*(((byte *)&x)+1))
#define LOBYTESGN(x) (*((signed char *)&x))
#define HIBYTESGN(x) (*((signed char *)&x))
#define BYTECAST(x) (byte)(x)
#define RX_ERR 	 	0xfe	//RX error
#define INS_ERR	 	0xfe	//instruction error
#define ACK_ERR 	0xfd	//I2C acknowledge error
#define SW_SPI	//some chips (such as 18F2550) have bugs in the hardware implementation
#define SW_I2C
#if defined(__18F2455)||defined(__18F2550)
	#define ID0	1
#elif defined(__18F2450)
	#define ID0	2
	#define SW_I2C
	#define NO_CCP2		//can't use CCP2 as source for ADC trigger
#elif defined(__18F2458)||defined(__18F2553)
	#define ID0	3
	#define ADC12		//12 bit ADC requires a different algorithm for DCDC control
#elif defined(__18F25K50)
	#define ID0	5       // gordoste variant firmware
#else
	#define ID0	255
#endif
#define Vcost 72.282 //(12/(12+22)/5*1024) //Voltage feedback constant

//***************PORTB*****************
#define Dnum 		5
#define Dlat 		LATB
#define Dport 		PORTB
#define Dtris 		TRISB
#define D_bit 		LATBbits.LATB5
#define Ddir_bit	TRISBbits.TRISB5
#define D0()  		D_bit=0
#define D1()  		D_bit=1
#define Din()  		Ddir_bit=1
#define Dout()  	Ddir_bit=0
#define CKnum 		6
#define CKlat 		LATB
#define CKport 		PORTB
#define CKtris 		TRISB
#define CK__bit 		LATBbits.LATB6
#define CKdir_bit	TRISBbits.TRISB6
#define CK0()  		CK__bit=0
#define CK1()  		CK__bit=1
#define CKin() 		CKdir_bit=1
#define CKout()		CKdir_bit=0
#define CKpulse()  	{CK__bit=1; CK__bit=0;}
#define CKpulseL() 	{Nop(); CK__bit=1; Nop(); Nop(); Nop(); CK__bit=0;}
#define CKpulseN()  {CK__bit=1; Nop(); CK__bit=0; Nop();}
#define PGMnum 		7
#define PGMlat 		LATB
#define PGMport 	PORTB
#define PGMtris 	TRISB
#define PGM__bit 	LATBbits.LATB7
#define PGMdir_bit	TRISBbits.TRISB7
#define PGM0()  	PGM__bit=0
#define PGM1()  	PGM__bit=1
#define PGMin() 	PGMdir_bit=1
#define PGMout()	PGMdir_bit=0
#define A2			LATBbits.LATB5
#define A1			LATBbits.LATB4
#define A0			LATBbits.LATB3
#define SCL			LATBbits.LATB1
#define SDA			LATBbits.LATB0
#define SDA_p		PORTBbits.RB0
#define A2_dir		TRISBbits.TRISB5
#define A1_dir		TRISBbits.TRISB4
#define A0_dir		TRISBbits.TRISB3
#define SCL_dir		TRISBbits.TRISB1
#define SDA_dir		TRISBbits.TRISB0
#define I2C_mask	0xC7	//11000111
#define SPI_mask	0xE5	//11100101
#define SPI_CS		LATBbits.LATB3
#define SPI_HLD		LATBbits.LATB4
#define SPICK_dir	TRISBbits.TRISB1
#define SPICK		LATBbits.LATB1
#define uWCK_dir	TRISBbits.TRISB1
#define uWCK		LATBbits.LATB1
#define uWCKnum		1
#define uWDI_dir	TRISBbits.TRISB0
#define uWDI		LATBbits.LATB0
#define uWDInum		0
#define uWDO_dir	TRISCbits.TRISC7
#define uWDO		LATCbits.LATC7
#define uWDOnum		7
#define PB1 		0
#define PB2 		1
#define PB0 		7
#define PB3 		6
#define TCKnum 		6
#define TDInum 		7
#define TDOnum 		5
#define TMSnum 		4
#define TCKpulse()  	{LABbits.LATB6=1; LABbits.LATB6=0;}

//***************PORTA*****************
#define LED1		LATAbits.LATA1
#define LED2		LATAbits.LATA2

//***************PORTC*****************
#define vcc_bit 	LATCbits.LATC1
#define vcc_dir 	TRISCbits.TRISC1
#define vcc_ON()	LATCbits.LATC1=0
#define vcc_OFF()	LATCbits.LATC1=1
#define vpp_bit 	LATCbits.LATC0
#define vpp_dir 	TRISCbits.TRISC0
#define vpp_ON()	LATCbits.LATC0=1
#define vpp_OFF()	LATCbits.LATC0=0
#define WP			LATCbits.LATC6
#define WP_dir		TRISCbits.TRISC6
#define SDO			LATCbits.LATC7
#define SDO_dir		TRISCbits.TRISC7

//***************PORTE*****************
#define SW1	 		PORTEbits.PORTE3

#if defined(SW_I2C)
#define  DATA_LOW   TRISBbits.TRISB0 = 0; // define macro for data pin output
#define  DATA_HI    TRISBbits.TRISB0 = 1; // define macro for data pin input
#define  DATA_LAT   LATBbits.LATB0        // define macro for data pin latch
#define  DATA_PIN   PORTBbits.RB0         // define macro for data pin
#define  CLOCK_LOW  TRISBbits.TRISB1 = 0; // define macro for clock pin output
#define  CLOCK_HI   TRISBbits.TRISB1 = 1; // define macro for clock pin input
#define  SCLK_LAT   LATBbits.LATB1        // define macro for clock pin latch
#endif

/** P U B L I C  P R O T O T Y P E S *****************************************/
void UserInit(void);
void ProcessIO(void);
void interrupt low_priority timer_isr (void);