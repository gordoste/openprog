/*********************************************************************
 * OPcontrol - control firmare for the Open Programmer
 * for more info see: openprog.altervista.org
 * Copyright (C) 2009-2020 Alberto Maccioni
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111 USA
 * or see <http://www.gnu.org/licenses/>
 *
*********************************************************************/

/*********************************************************************
History
0.3.0  - initial release; PIC10-12-16-18
0.4.0  - 1/8/08 added I2C instructions
0.5.0  - 10/11/08 added SPI, ATMEL instructions
0.6.0  - 13/6/09 switched to a GPL USB framework, added uW and PIC24 ins.
0.6.1  - 30/8/09 added software SPI
0.7.0  - 31/12/09 added and modified PIC24 instructions
0.7.6  - 1/7/10 modified TX16 & RX16 instructions, conditional code to use 12 bit ADC on 18F2553
0.8.0  - 30/6/12 added One Wire support, auto-flush tx buffer when rx all processed, fixed PROG_C
0.9.0  - 25/1/14 added SET_PORT_DIR,AT_HV_RTX,SIX_LONG5; improved DCDC control
0.10.0 - 11/6/16 added LOAD_PC, LOAD_DATA_INC, READ_DATA_INC, JTAG_SET_MODE, JTAG_SEND_CMD, JTAG_XFER_DATA, JTAG_XFER_F_DATA, 
				 new USB VID&PID (0x1209,0x5432), changed some CK timing, reduced CLOCK_GEN startup time
0.11.0 - 14/1/19 added ICSP8_SHORT,ICSP8_READ,ICSP8_LOAD
0.11.2 - 23/9/20 extended ICSP8_READ payload to 16 bits (for PIC18) 
*********************************************************************
Map of peripherals

Timer0	blink timer: 11 ms

Timer2	(if DCDC on): 90kHz, no prescaler, no postscaler

Timer3 	(if DCDC on): synchronous counter, no prescaler, source for CCP2
		(after CLOCK_GEN command): 16 bit, source for CCP1 & CCP2, no prescaler

CCP1	(if DCDC on): PWM mode, clock from timer2, 90 kHz, to DCDC converter
		(after CLOCK_GEN command): compare mode, reset timer3 on match

CCP2	(if DCDC on): compare mode, trigger ADC every 250us
		(after CLOCK_GEN command): compare mode, toggle on match, clock to external devices

ADC: 	acquires Vreg*12k/34k on AN0, FOSC/64, triggered by CCP2, generates interrupt

MSSP 	(I2C mode): master
		the following was removed for problems with the hardware peripheral:
		[(SPI mode): master, clock from timer2 ]

If compiled for 18F2450:
Timer1  (if DCDC on): interrupt every 250us, ADC started by interrupt routine;
no CCP2
no MSSP
software I2C
software SPI

*********************************************************************/




/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "usb.h"
#include "OPcontrol.h"
#include "instructions.h"

//********Misc.********
#define VERSION "0.11.2"
#define VER2	0
#define VER1	11
#define VER0	2
#define ID2	0
#define ID1	0
#define TX_TEMP_MAX 10
#define LOBYTE(x) (*((char *)&x))
#define HIBYTE(x) (*(((char *)&x)+1))
#define RX_ERR 	 	0xfe	//RX error
#define INS_ERR	 	0xfe	//instruction error
#define ACK_ERR 	0xfd	//I2C acknowledge error
#define SW_SPI	//some chips (such as 18F2550) have bugs in the hardware implementation
#if defined(__18F2455)||defined(__18F2550)
	#define ID0	1
#elif defined(__18F2450)
	#define ID0	2
	#define SW_I2C
	#define NO_CCP2		//can't use CCP2 as source for ADC trigger
#elif defined(__18F2458)||defined(__18F2553)
	#define ID0	3
	#define ADC12		//12 bit ADC requires a different algorithm for DCDC control
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
#define CK_bit 		LATBbits.LATB6
#define CKdir_bit	TRISBbits.TRISB6
#define CK0()  		CK_bit=0
#define CK1()  		CK_bit=1
#define CKin() 		CKdir_bit=1
#define CKout()		CKdir_bit=0
#define CKpulse()  	{CK_bit=1; CK_bit=0;}
#define CKpulseL() 	{Nop(); CK_bit=1; Nop(); Nop(); Nop(); CK_bit=0;}
#define CKpulseN()  {CK_bit=1; Nop(); CK_bit=0; Nop();}
#define PGMnum 		7
#define PGMlat 		LATB
#define PGMport 	PORTB
#define PGMtris 	TRISB
#define PGM_bit 	LATBbits.LATB7
#define PGMdir_bit	TRISBbits.TRISB7
#define PGM0()  	PGM_bit=0
#define PGM1()  	PGM_bit=1
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

/** V A R I A B L E S ********************************************************/

#if !defined(__18F2450)
#pragma romdata eedata=0xF00000
rom char eestr[]="Open Programmer v. ";
rom char eestr2[]=VERSION;
rom char eestr3[]=" - Copyright (C) 2009-2020 Alberto Maccioni - This is free software";
#endif

#pragma udata

char transmit_buffer[HID_OUTPUT_REPORT_BYTES];
char receive_buffer[HID_INPUT_REPORT_BYTES];
char transmit_temp[TX_TEMP_MAX];
byte number_of_bytes_read=0;
byte IN_pending;
byte led_cnt;
byte T2,M,N;
int T3,timeout;

#pragma idata access my_access
/* all accesses to these will be unbanked */
near unsigned char RXptr=0,TXptr=0,TXaux=0;
near int pwm=0;
near byte pwm_maxH=100,T1=1;
near int volatile err=0,errz=0,vreg=13.0*Vcost;
near byte i=0,i2=0,RES0=0,RES1=0,RES2=0,RES3=0;
near int d=0,dH=0; 
#if defined(SW_SPI)||defined(SW_I2C)
near unsigned char I2C_BUFFER=0;    // temp buffer for R/W operations
near unsigned char BIT_COUNTER=0;   // temp buffer for bit counting
near byte tt=0;
#endif

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void TXins(byte x);
void ParseCommands(void);
void BlinkStatus();
void Delay2us(unsigned char delay);
void Delay1us(unsigned char delay);
#if defined(SW_I2C)
void SWStopI2C( void );                // Generate bus stop condition
void SWStartI2C( void );               // Generate bus start condition
void SWRestartI2C( void );             // Generate bus restart condition
signed char SWAckI2C( void );             // Read bus ACK condition
unsigned int  SWGetcI2C( void );          // Read in single byte
signed char SWPutcI2C( auto unsigned char data_out ); // Write out single byte
void NAckI2C( void );
void AckI2C( void );
void DelayT();
#define  DATA_LOW   TRISBbits.TRISB0 = 0; // define macro for data pin output
#define  DATA_HI    TRISBbits.TRISB0 = 1; // define macro for data pin input
#define  DATA_LAT   LATBbits.LATB0        // define macro for data pin latch
#define  DATA_PIN   PORTBbits.RB0         // define macro for data pin
#define  CLOCK_LOW  TRISBbits.TRISB1 = 0; // define macro for clock pin output
#define  CLOCK_HI   TRISBbits.TRISB1 = 1; // define macro for clock pin input
#define  SCLK_LAT   LATBbits.LATB1        // define macro for clock pin latch
#endif
#if defined(SW_SPI)
unsigned char SW_IO_SPI(unsigned char c);	//transfer one byte
#endif

/** D E C L A R A T I O N S **************************************************/

#pragma code
void UserInit(void)
{
	IN_pending=0;
	led_cnt=0;
	LATB=0;
	LATA=0;
	TRISA=0b11111001;
	ADCON0=0x01;			//AN0, ADC ON
	ADCON1=0x0E;			//AN0, internal ref
	ADCON2=0b10000110;		//LSB, 0Tad, FOSC/64
#if !defined(NO_CCP2)
	CCP1CON=CCP2CON=0;
#else
	CCP1CON=0;
#endif
	T1=1;
	T2=100;
	T3=2000;
	M=8;
	N=11;
	err=errz=pwm=0;
    T0CON=0x80;				// Timer0 period = T*2^16:2 = 5.46ms:2 = 10.93ms (PS=2)
}

void BlinkStatus(){
	if(INTCONbits.TMR0IF){
		INTCONbits.TMR0IF=0;
		led_cnt++;
		if (deviceState<CONFIGURED&&led_cnt>=10){
			LED2 = !LED2;
			led_cnt=0;
		}
		else if (deviceState == CONFIGURED&&led_cnt>=46){
			LED2 = !LED2;
			led_cnt=0;
		}
	}
}


/******************************************************************************
 * Function:        void ProcessIO(void)
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        This function is a place holder for other user routines.
 *                  It is a mixture of both USB and non-USB tasks.
 * Note:            None
 *****************************************************************************/

void ProcessIO(void)
{
	// Called repeatedly in main().

    BlinkStatus();

    // User Application USB tasks

    if((deviceState < CONFIGURED)||(UCONbits.SUSPND==1)) return;

    if(IN_pending&&!(ep1Bi.Stat&UOWN)){
	    HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
	    IN_pending=0;
		for(TXptr=0;TXptr<TXaux;TXptr++) transmit_buffer[TXptr]=transmit_temp[TXptr];
		TXaux=0;
	}
	if(RXptr>=number_of_bytes_read){			//RXptr>max
		if(TXptr){ 		//send report once if non empty when rx is all processed
			for(;TXptr<HID_INPUT_REPORT_BYTES;TXptr++) transmit_buffer[TXptr]=0;
			TXptr=0;
			if(!(ep1Bi.Stat&UOWN)) HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
			else IN_pending=1;
		}
 		number_of_bytes_read = HIDRxReport(receive_buffer, HID_OUTPUT_REPORT_BYTES);
		if (number_of_bytes_read) RXptr=0;
	}
	ParseCommands();
}

/******************************************************************************
 * Function:        void ParseCommands(void)
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Instruction parser
 * Note:            None
 *****************************************************************************/

void ParseCommands(void)
{
	//total overhead between instructions is 30us (not counting DCDC control function)
	if (RXptr<number_of_bytes_read&&!IN_pending){
		LED1=1;
		switch(receive_buffer[RXptr]){
			case NOP:		//NOP
				TXins(NOP);
				break;
			case PROG_RST:		//Reset, 10 bytes [32us]
				TXins(PROG_RST);
				UserInit();
				TXins(VER2);
				TXins(VER1);
				TXins(VER0);
				TXins(ID2);
				TXins(ID1);
				TXins(ID0);
				TXins(' ');
				TXins('R');
				TXins('S');
				TXins('T');
				break;
			case PROG_ID:		//ID, 6 bytes [18us]
				TXins(PROG_ID);
				TXins(VER2);
				TXins(VER1);
				TXins(VER0);
				TXins(ID2);
				TXins(ID1);
				TXins(ID0);
				break;
			case CHECK_INS:		//check ins [7us]
				TXins(CHECK_INS);
				if(RXptr+1<number_of_bytes_read){
					byte x=receive_buffer[++RXptr];
					if(x<MAX_INS) TXins(x);
					else TXins(INS_ERR);
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case FLUSH:		//Flush buffers (in&out)
				for(;TXptr<HID_INPUT_REPORT_BYTES;TXptr++) transmit_buffer[TXptr]=0;
				TXptr=0;
   	   			if(!(ep1Bi.Stat&UOWN)) HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
   	 			else IN_pending=1;
				RXptr=number_of_bytes_read;
				break;
			case VREG_EN:			//enable DCDC [5us]
				TXins(VREG_EN);
				err=errz=pwm=0;
				PR2=0x84;			//PWM period=11us f=90kHz
				T2CON = 4;			//timer2 ON
#if defined(NO_CCP2)				//use timer1 if CCP2 is not present
				PIR1bits.TMR1IF=0;
				PIE1bits.TMR1IE=1;	//enable Timer1 interrupt
				TMR1H=0xF4;			//64K-3000 @48MHz = 250 us
				TMR1L=0x48;
				T1CON=0b10000001;	//timer1, 16 bit, no prescaler
#else								//else use CCP2
				CCPR2H=0x0B;		//CCPR2=3000 (0xBB8) 12MHz/3000= 4KHz = 250us
				CCPR2L=0xB8;
				PIR1bits.ADIF=0;
				PIE1bits.ADIE=1;	//enable ADC interrupt
				TMR3L=TMR3H=0;
				T3CON=0b10001001;	//timer3, 16 bit, linked to CCP2, no prescaler
				CCP2CON=0x0B;		//enable compare mode with ADC trigger
#endif
				CCPR1L=0;			//SetDCPWM1(0);
				CCP1CON=0x0C;		//enable pwm
				TRISCbits.TRISC2=0;	//PWM1 out
				INTCON=0b11000000;	//enable interrupt
				HLVDCON=0b00011110;	//enable power supply comparator @ 4.5V
				break;
			case VREG_DIS:			//disable DCDC
				TXins(VREG_DIS);
#if defined(NO_CCP2)
				PIE1bits.TMR1IE=0;	//disable Timer1 interrupt
				CCP1CON=T1CON=0;	//timer1 off, pwm off
#else								//others use CCP2
				PIE1bits.ADIE=0;	//ADC interrupt
				CCP1CON=CCP2CON=0;	//pwm off
#endif
				HLVDCON=0b00001110;	//disable power supply comparator
				break;
			case SET_PARAMETER:
				TXins(SET_PARAMETER);
				if(RXptr+3<number_of_bytes_read){
					i=receive_buffer[++RXptr];
					switch(i){
						case SET_T1T2:				//T1-T2
							T1=receive_buffer[++RXptr];
							T2=receive_buffer[++RXptr];
						break;
						case SET_T3:				//T3
							HIBYTE(T3)=receive_buffer[++RXptr];
							LOBYTE(T3)=receive_buffer[++RXptr];
						break;
						case SET_timeout:			//timeout
							HIBYTE(timeout)=receive_buffer[++RXptr];
							LOBYTE(timeout)=receive_buffer[++RXptr];
						break;
						case SET_MN:				//M e N
							M=receive_buffer[++RXptr];
							N=receive_buffer[++RXptr];
						break;
						default:
							RXptr+=2;
						break;
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case WAIT_T1:		//delay T1 us
				TXins(WAIT_T1);
				INTCONbits.GIE=0;
				for(i=T1;i;i--){
					Nop();
					Nop();
					Nop();
					Nop();
					Nop();
					Nop();
					Nop();
				}
				INTCONbits.GIE=1;
				break;
			case WAIT_T2:		//delay T2 us
				TXins(WAIT_T2);
				INTCONbits.GIE=0;
				for(i=T2;i;i--){
					Nop();
					Nop();
					Nop();
					Nop();
					Nop();
					Nop();
					Nop();
				}
				INTCONbits.GIE=1;
				break;
			case WAIT_T3:		//delay T3 us (16 bit)
				TXins(WAIT_T3);
				INTCONbits.GIE=0;
				for(d=T3;d;d--){
					Nop();
					Nop();
					Nop();
					Nop();
				}
				INTCONbits.GIE=1;
				break;
			case WAIT_US:		//manual delay us (8 bit)
				TXins(WAIT_US);
				if(RXptr+1<number_of_bytes_read){
					for(i=receive_buffer[++RXptr];i;i--){
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case READ_ADC:		//read ADC
				PIR2bits.HLVDIF=0;
				TXins(READ_ADC);
				ADCON0bits.GO=1;
				while(ADCON0bits.GO);
				if(PIR2bits.HLVDIF){
					TXins(0);
					TXins(0);
					break;
				}
				TXins(ADRESH);
				TXins(ADRESL);
				break;
			case SET_VPP:
			//set vpp (X x 0.1V) and wait for it to stabilize to +- 0.2V
			//return X if successful, else INS_ERR
				TXins(SET_VPP);
				if(RXptr+1<number_of_bytes_read){
#if !defined(NO_CCP2)
					if(!PIE1bits.ADIE||!HLVDCONbits.IVRST) TXins(INS_ERR); //Check interrupt and HLVD
#else
					if(!PIE1bits.TMR1IE||!HLVDCONbits.IVRST) TXins(INS_ERR); //Check interrupt and HLVD
#endif
					else{
						PIR2bits.HLVDIF=0;
						d=(unsigned char)receive_buffer[++RXptr];
						vreg=d*72/10;//7;
						i=15;			//15 ms
						do{
							for(d=1000;d;d--){			//to update err
								Nop();
								Nop();
								Nop();
								Nop();
							}
							i--;
						}
						while((err>14||err<-14)&&i);
						if(PIR2bits.HLVDIF) TXins(0);
						else if(err>14||err<-14) TXins(INS_ERR);
						else TXins(receive_buffer[RXptr]);
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case EN_VPP_VCC:
			//controls VPP and VCC, bit 0 VCC, bit 2 VPP
				TXins(EN_VPP_VCC);
				if(RXptr+1<number_of_bytes_read){
					i=receive_buffer[++RXptr];
					vcc_bit=(i&0x01)?0:1;
					vcc_dir=(i&0x02)?1:0;
					vpp_bit=(i&0x04)?1:0;
					vpp_dir=(i&0x08)?1:0;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SET_CK_D:
				//forces CK, D, PGM in 2 bits:
				//bit 0= D level, bit 1=D impedance, bit 2= CK level, bit 3=CK imp.
				//bit 4= PGM level, bit 5=PGM imp.
				TXins(SET_CK_D);
				if(RXptr+1<number_of_bytes_read){
					D_bit=(receive_buffer[++RXptr]&1)?1:0;
					Ddir_bit=(receive_buffer[RXptr]&2)?1:0;
					CK_bit=(receive_buffer[RXptr]&4)?1:0;
					CKdir_bit=(receive_buffer[RXptr]&8)?1:0;
					PGM_bit=(receive_buffer[RXptr]&16)?1:0;
					PGMdir_bit=(receive_buffer[RXptr]&32)?1:0;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case READ_PINS:
				//read CK, D, PGM:
				//bit 0= D level, bit 1=D impedance, bit 2= CK level, bit 3=CK imp.
				//bit 4= PGM level, bit 5=PGM imp.
				TXins(READ_PINS);
				i=0;
				if(PORTBbits.RB5) i|=1;	//D_bit
				if(Ddir_bit) i|=2;
				if(PORTBbits.RB6) i|=4;	//CK_bit
				if(CKdir_bit) i|=8;
				if(PORTBbits.RB7) i|=16;	//PGM_bit
				if(PGMdir_bit) i|=32;
				TXins(i);
				break;
			case LOAD_CONF:				//Load configuration 000000, 14 bit data [22us]
				TXins(LOAD_CONF);
				if(RXptr+2<number_of_bytes_read){
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					D0();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
					CKpulseN();		//start bit
					_asm
					MOVLW	14
					movwf	i,0
					ciclo_c1:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_c1
					_endasm
					D0();
					CKpulseN();	//Stop bit
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case LOAD_DATA_PROG:			//Load data in program memory 000010, 14 bit data [22us]
				TXins(LOAD_DATA_PROG);
				if(RXptr+2<number_of_bytes_read){
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					D0();
					CKpulseN();
					D1();
					CKpulseN();
					D0();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
					CKpulseN();		//start bit
					_asm
					MOVLW	14
					movwf	i,0
					ciclo_l1:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_l1
					_endasm
					D0();
					CKpulseN();	//Stop bit
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case LOAD_DATA_DATA:			//Load data in data memory 000011, 8 bit data [17us]
				TXins(LOAD_DATA_DATA);
				if(RXptr+1<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					D1();
					CKpulseN();
					CKpulseN();
					D0();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
					CKpulseN();		//start bit
					_asm
					MOVLW	8
					movwf	i,0
					ciclo_l2:
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_l2
					_endasm
					D0();
					CKpulseN();	//bit 10
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();	//MSB
					CKpulseN();	//Stop bit
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case READ_DATA_PROG:			//Read data from program memory 000100 [27us]
				TXins(READ_DATA_PROG);
				INTCONbits.GIE=0;
				D0();
				CKpulseN();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				CKpulseN();
				Ddir_bit=1;		//Input
				for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
				CKpulseN();		//start bit
				_asm
				MOVLW	14
				movwf	i,0
				ciclo_r1:
				BSF 	LATB,CKnum,0
				nop
				BCF 	LATB,CKnum,0
				BCF 	STATUS,0,0		//Carry
				BCF 	LATB,CKnum,0
				BTFSC	PORTB,Dnum,0
				BSF 	STATUS,0,0		//Carry
				RRCF	d+1,1,0
				RRCF	d,1,0
				decfsz	i,1,0
				BRA ciclo_r1
				BCF 	STATUS,0,0		//Carry
				RRCF	d+1,1,0
				RRCF	d,1,0
				BCF 	STATUS,0,0		//Carry
				RRCF	d+1,1,0
				RRCF	d,1,0
				_endasm
				CK1();	//Stop bit
				Ddir_bit=0;		//Output
				CK0();	//Stop bit
				INTCONbits.GIE=1;
				TXins(HIBYTE(d));
				TXins(LOBYTE(d));
				break;
			case READ_DATA_DATA:			//Read data from data memory 000101 [19us]
				TXins(READ_DATA_DATA);
				INTCONbits.GIE=0;
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				CKpulseN();
				Ddir_bit=1;		//Input
				for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
				CKpulseN();		//start bit
				_asm
				MOVLW	8
				movwf	i,0
				ciclo_r3:
				BSF 	LATB,CKnum,0
				nop
				BCF 	LATB,CKnum,0
				BCF 	STATUS,0,0		//Carry
				BTFSC	PORTB,Dnum,0
				BSF 	STATUS,0,0		//Carry
				RRCF	d,1,0
				decfsz	i,1,0
				BRA ciclo_r3
				_endasm
				CKpulseN();	//bit 10
				CKpulseN();
				CKpulseN();
				CKpulseN();
				CKpulseN();
				CKpulseN();	//MSB
				CK1();	//Stop bit
				Ddir_bit=0;		//Output
				CK0();	//Stop bit
				INTCONbits.GIE=1;
				TXins(LOBYTE(d));
				break;
			case INC_ADDR:				//Increment address 000110 [5us]
				TXins(INC_ADDR);
				INTCONbits.GIE=0;
				D0();
				CKpulseN();
				D1();
				CKpulseN();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				CKpulseN();
				INTCONbits.GIE=1;
				break;
			case INC_ADDR_N:			//Increment address 000110 (N times) [(4+3.3*N)us]
				TXins(INC_ADDR_N);
				if(RXptr+1<number_of_bytes_read){
					INTCONbits.GIE=0;
					for(LOBYTE(d)=receive_buffer[++RXptr];LOBYTE(d);LOBYTE(d)--){
						D0();
						CKpulseN();
						D1();
						CKpulseN();
						CKpulseN();
						D0();
						CKpulseN();
						CKpulseN();
						CKpulseN();
						for(i=T1;i;i--){	//T1 us
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
						}
					}
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case BEGIN_PROG:			//Begin programming 001000 [4.5us]
				TXins(BEGIN_PROG);
				INTCONbits.GIE=0;
				D0();
				CKpulseN();
				CKpulseN();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				INTCONbits.GIE=1;
				break;
			case BULK_ERASE_PROG:		//Bulk erase program memory 001001 [4.5us]
				TXins(BULK_ERASE_PROG);
				INTCONbits.GIE=0;
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				INTCONbits.GIE=1;
				break;
			case END_PROG:				//End programming 001010 [4.5us]
				TXins(END_PROG);
				INTCONbits.GIE=0;
				D0();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				INTCONbits.GIE=1;
				break;
			case BULK_ERASE_DATA:		//Bulk erase data memory 001011 [4.5us]
				TXins(BULK_ERASE_DATA);
				INTCONbits.GIE=0;
				D1();
				CKpulseN();
				CKpulseN();
				D0();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				INTCONbits.GIE=1;
				break;
			case END_PROG2:				//End programming 2 001110 [4.5us]
				TXins(END_PROG2);
				INTCONbits.GIE=0;
				D0();
				CKpulseN();
				D1();
				CKpulseN();
				CKpulseN();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				INTCONbits.GIE=1;
				break;
			case ROW_ERASE_PROG:		//Row erase program memory 010001 [4.5us]
				TXins(ROW_ERASE_PROG);
				INTCONbits.GIE=0;
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				INTCONbits.GIE=1;
				break;
			case BEGIN_PROG2:			//Begin programming 2 011000 [4.5us]
				TXins(BEGIN_PROG2);
				INTCONbits.GIE=0;
				D0();
				CKpulseN();
				CKpulseN();
				CKpulseN();
				D1();
				CKpulseN();
				CKpulseN();
				D0();
				CKpulseN();
				INTCONbits.GIE=1;
				break;
			case CUST_CMD:			//Custom command [9us]
				TXins(CUST_CMD);
				if(RXptr+1<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					_asm
					MOVLW	6
					movwf	i,0
					ciclo_c2:
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_c2
					_endasm
					D0();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case PROG_C:
			//Program and verify word; 000010, 001000, 001110, M pulses & Nx overpulses; 14 bit data
				TXins(PROG_C);
				if(RXptr+2<number_of_bytes_read){
					int t,j;
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					t=d;
					INTCONbits.GIE=0;
					D0();		//load data 000010
					CKpulseN();
					D1();
					CKpulseN();
					D0();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
					CKpulseN();		//start bit
					_asm
					MOVLW	14
					movwf	i,0
					ciclo_p1:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_p1
					_endasm
					D0();
					CKpulseN();	//Stop bit
					for(j=0;j<M;j++){
						D0();	//001000 BEGIN PROG
						CKpulseN();
						CKpulseN();
						CKpulseN();
						D1();
						CKpulseN();
						D0();
						CKpulseN();
						CKpulseN();
						for(i=T2;i;i--){	//100us
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
						}
						D0();	//End programming 2 001110
						CKpulseN();
						D1();
						CKpulseN();
						CKpulseN();
						CKpulseN();
						D0();
						CKpulseN();
						CKpulseN();
						for(i=T1;i;i--){	//1us
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
						}
						D0();	//READ_PROG    000100
						CKpulseN();
						CKpulseN();
						D1();
						CKpulseN();
						D0();
						CKpulseN();
						CKpulseN();
						CKpulseN();
						Ddir_bit=1;		//Input
						for(i=T1;i;i--){	//T1 us
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
							}
						CKpulseN();		//start bit
						_asm
						MOVLW	14
						movwf	i,0
						ciclo_p2:
						BSF 	LATB,CKnum,0
						nop
						BCF 	LATB,CKnum,0
						BCF 	STATUS,0,0		//Carry
						BTFSC	PORTB,Dnum,0
						BSF 	STATUS,0,0		//Carry
						RRCF	d+1,1,0
						RRCF	d,1,0
						decfsz	i,1,0
						BRA ciclo_p2
						BCF 	STATUS,0,0		//Carry
						RRCF	d+1,1,0
						RRCF	d,1,0
						BCF 	STATUS,0,0		//Carry
						RRCF	d+1,1,0
						RRCF	d,1,0
						_endasm
						CK1();	//Stop bit
						Ddir_bit=0;		//Output
						CK0();	//Stop bit
						for(i=T1;i;i--){	//1us
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
						}
						if(t==d){
							i=j+1;
							j=0xff;
						}
					}
					INTCONbits.GIE=1;
					INTCONbits.GIE=0;
					if(j==0x100){
						TXins(i);
						for(j=i*N;j;j--){
							D0();	//001000 BEGIN PROG
							CKpulseN();
							CKpulseN();
							CKpulseN();
							D1();
							CKpulseN();
							D0();
							CKpulseN();
							CKpulseN();
							for(i=T2;i;i--){	//100us
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
							}
							D0();	//End programming 2 001110
							CKpulseN();
							D1();
							CKpulseN();
							CKpulseN();
							CKpulseN();
							D0();
							CKpulseN();
							CKpulseN();
							for(i=T1;i;i--){	//1us
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
								Nop();
							}
						}
					}
					else TXins(INS_ERR);
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case LOAD_PC:				//Load PC address 011101, 16 bit data (tot 6+24 pulses)[26us]
				TXins(LOAD_PC);
				if(RXptr+2<number_of_bytes_read){
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					D1();
					CKpulseN();
					D0();
					CKpulseN();
					D1();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					D0();
					CKpulseN();
					for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
					CKpulseN();		//start bit
					_asm
					MOVLW	16
					movwf	i,0
					ciclo_c3:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_c3
					_endasm
					D0();
					CKpulseN();	//Stop bit
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case LOAD_DATA_INC:			//Load data in NVM memory and increase address 100010, 14 bit data [22us]
				TXins(LOAD_DATA_INC);
				if(RXptr+2<number_of_bytes_read){
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					D0();
					CKpulseN();
					D1();
					CKpulseN();
					D0();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					D1();
					CKpulseN();
					for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
					D0();
					CKpulseN();		//start bit
					_asm
					MOVLW	14
					movwf	i,0
					ciclo_l3:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_l3
					_endasm
					D0();
					CKpulseN();	//Stop bit
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case READ_DATA_INC:			//Read data from NVM memory and increase address 100100 [29us]
				TXins(READ_DATA_INC);
				INTCONbits.GIE=0;
				D0();
				CKpulseN();
				CKpulseN();
				D1();
				CKpulseN();
				D0();
				CKpulseN();
				CKpulseN();
				D1();
				CKpulseN();
				Ddir_bit=1;		//Input
				for(i=T1;i;i--){	//T1 us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
				D0();
				CKpulseN();		//start bit
				_asm
				MOVLW	14
				movwf	i,0
				ciclo_r4:
				BSF 	LATB,CKnum,0
				nop
				BCF 	LATB,CKnum,0
				BCF 	STATUS,0,0		//Carry
				BTFSC	PORTB,Dnum,0
				BSF 	STATUS,0,0		//Carry
				RRCF	d+1,1,0
				RRCF	d,1,0
				decfsz	i,1,0
				BRA ciclo_r4
				BCF 	STATUS,0,0		//Carry
				RRCF	d+1,1,0
				RRCF	d,1,0
				BCF 	STATUS,0,0		//Carry
				RRCF	d+1,1,0
				RRCF	d,1,0
				_endasm
				CK1();	//Stop bit
				Ddir_bit=0;		//Output
				CK0();	//Stop bit
				INTCONbits.GIE=1;
				TXins(HIBYTE(d));
				TXins(LOBYTE(d));
				break;
			case CORE_INS:				//Core instruction (PIC18) 0000, 16 bit data [22us]
				TXins(CORE_INS);
				if(RXptr+2<number_of_bytes_read){
					INTCONbits.GIE=0;
					CK1();
					Ddir_bit=0;		//Output
					D0();
					CK0();
					CKpulseN();
					CKpulseN();
					CKpulseN();
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					_asm
					MOVLW	16
					movwf	i,0
					ciclo_r181:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r181
					fine_r181:
					_endasm
					D0();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SHIFT_TABLAT:				//Shift TABLAT (PIC18) 0010, 8 bit data [15us]
				TXins(SHIFT_TABLAT);
				INTCONbits.GIE=0;
				CK1();
				Ddir_bit=0;		//Output
				D0();
				CK0();
				D1();
				CKpulse();
				D0();
				CKpulse();
				CKpulse();		//fine ID
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				Ddir_bit=1;		//Input
				_asm
				MOVLW	8
				movwf	i,0
				ciclo_r182:
				BSF 	LATB,CKnum,0	//CKpulse();
				BCF 	LATB,CKnum,0
				BCF 	STATUS,0,0		//Carry
				BTFSC 	PORTB,Dnum,0
				BSF 	STATUS,0,0		//Carry
				RRCF	d,1,0
				decfsz	i,1,0
				BRA 	ciclo_r182
				fine_r182:
				_endasm
				D0();
				INTCONbits.GIE=1;
				TXins(LOBYTE(d));
				break;
			case TABLE_READ:				//Table read (PIC18) 1000, 8 bit data [15us]
				TXins(TABLE_READ);
				INTCONbits.GIE=0;
				CK1();
				Ddir_bit=0;		//Output
				D0();
				CK0();
				CKpulse();
				CKpulse();
				D1();
				CKpulse();		//fine ID
				D0();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				CKpulse();
				Ddir_bit=1;		//Input
				_asm
				MOVLW	8
				movwf	i,0
				ciclo_r183:
				BSF 	LATB,CKnum,0	//CKpulse();
				BCF 	LATB,CKnum,0
				BCF 	STATUS,0,0		//Carry
				BTFSC 	PORTB,Dnum,0
				BSF 	STATUS,0,0		//Carry
				RRCF	d,1,0
				decfsz	i,1,0
				BRA 	ciclo_r183
				_endasm
				D0();
				INTCONbits.GIE=1;
				TXins(LOBYTE(d));
				break;
			case TBLR_INC_N:				//Table read post-inc (PIC18) 1001, 8 bit data, N times [(7+N*12)us]
				TXins(TBLR_INC_N);
				i=receive_buffer[++RXptr];
				if(RXptr<number_of_bytes_read&&TXptr+i<HID_INPUT_REPORT_BYTES){
					byte j;
					TXins(i);
					for(j=i;j;j--){
						INTCONbits.GIE=0;
						CK1();
						Ddir_bit=0;		//Output
						D1();
						CK0();
						D0();
						CKpulse();
						CKpulse();
						D1();
						CKpulse();		//fine ID
						D0();
						CKpulse();
						CKpulse();
						CKpulse();
						CKpulse();
						CKpulse();
						CKpulse();
						CKpulse();
						CKpulse();
						Ddir_bit=1;		//Input
						_asm
						MOVLW	8
						movwf	i,0
						ciclo_r184:
						BSF 	LATB,CKnum,0	//CKpulse();
						BCF 	LATB,CKnum,0
						BCF 	STATUS,0,0		//Carry
						BTFSC 	PORTB,Dnum,0
						BSF 	STATUS,0,0		//Carry
						RRCF	d,1,0
						decfsz	i,1,0
						BRA 	ciclo_r184
						_endasm
						D0();
						INTCONbits.GIE=1;
						TXins(LOBYTE(d));
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case TABLE_WRITE:				//Table write (PIC18) 1100, 16 bit data [20us]
				TXins(TABLE_WRITE);
				if(RXptr+2<number_of_bytes_read){
					INTCONbits.GIE=0;
					CK1();
					Ddir_bit=0;		//Output
					D0();
					CK0();
					CKpulse();
					D1();
					CKpulse();
					CKpulse();
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					_asm
					MOVLW	16
					movwf	i,0
					ciclo_r187:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulse();
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r187
					_endasm
					D0();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case TBLW_INC_N:				//Table write post-inc (PIC18) 1101, 16 bit data, N times [(4+N*17)us]
				TXins(TBLW_INC_N);
				i=receive_buffer[++RXptr];
				if(RXptr+i+i<number_of_bytes_read){
					byte j;
					for(j=i;j;j--){
						INTCONbits.GIE=0;
						CK1();
						Ddir_bit=0;		//Output
						D1();
						CK0();
						D0();
						CKpulse();
						D1();
						CKpulse();
						CKpulse();
						HIBYTE(d)=receive_buffer[++RXptr];
						LOBYTE(d)=receive_buffer[++RXptr];
						_asm
						MOVLW	16
						movwf	i,0
						ciclo_r188:
						RRCF	d+1,1,0
						RRCF	d,1,0
						BCF 	LATB,Dnum,0	//D0();
						BTFSC 	STATUS,0,0		//Carry
						BSF 	LATB,Dnum,0	//D1();
						BSF 	LATB,CKnum,0	//CKpulse();
						BCF 	LATB,CKnum,0
						decfsz	i,1,0
						BRA 	ciclo_r188
						_endasm
						D0();
						INTCONbits.GIE=1;
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case TBLW_PROG:				//Table write and program (PIC18) 1111, 16 bit data, wait XX us [(25+T2+N)us]
				TXins(TBLW_PROG);
				if(RXptr+4<number_of_bytes_read){
					INTCONbits.GIE=0;
					CK1();
					Ddir_bit=0;		//Output
					D1();
					CK0();
					CKpulse();
					CKpulse();
					CKpulse();
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					_asm
					MOVLW	16
					movwf	i,0
					ciclo_r189:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulse();
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r189
					_endasm
					D0();
					CKpulse();			//NOP + wait XX us
					CKpulse();
					CKpulse();
					CK1();
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					for(;d;d--){
						Nop();
						Nop();
						Nop();
						Nop();
					}
					CK0();
					for(i=T2;i;i--){
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
					for(i=16;i;i--){
						CKpulse();
					}
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case TBLW_PROG_INC:				//Table write, program, inc. address (PIC18) 1110, 16 bit data, wait XX us [(25+T2+N)us]
				TXins(TBLW_PROG_INC);
				if(RXptr+4<number_of_bytes_read){
					INTCONbits.GIE=0;
					CK1();
					Ddir_bit=0;		//Output
					D0();
					CK0();
					D1();
					CKpulse();
					CKpulse();
					CKpulse();
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					_asm
					MOVLW	16
					movwf	i,0
					ciclo_r190:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulse();
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r190
					_endasm
					D0();
					CKpulse();			//NOP + wait XX us
					CKpulse();
					CKpulse();
					CK1();
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					for(;d;d--){
						Nop();
						Nop();
						Nop();
						Nop();
					}
					CK0();
					for(i=T2;i;i--){
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
					}
					for(i=16;i;i--){
						CKpulse();
					}
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SEND_DATA:				//Send data (PIC18) , (0000+) 4 bit command ID, 16 bit data [25us]
				TXins(SEND_DATA);
				if(RXptr+3<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];	//ID 4 bit
					INTCONbits.GIE=0;
					CK1();
					Ddir_bit=0;		//Output
					_asm
					MOVLW	4
					movwf	i,0
					ciclo_r191:
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulse();
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r191
					_endasm
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					_asm
					MOVLW	16
					movwf	i,0
					ciclo_r192:
					RRCF	d+1,1,0
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulse();
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r192
					_endasm
					D0();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case READ_DATA:				//Read data (PIC18) , (0000+) 4 bit command ID, 8 bit data [20us]
				TXins(READ_DATA);
				if(RXptr+1<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];	//ID 4 bit
					INTCONbits.GIE=0;
					CK1();
					Ddir_bit=0;		//Output
					_asm
					MOVLW	4
					movwf	i,0
					ciclo_r193:
					RRCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulse();
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r193
					_endasm
					CKpulse();
					CKpulse();
					CKpulse();
					CKpulse();
					CKpulse();
					CKpulse();
					CKpulse();
					CKpulse();
					Ddir_bit=1;		//Input
					_asm
					MOVLW	8
					movwf	i,0
					ciclo_r194:
					BSF 	LATB,CKnum,0	//CKpulse();
					BCF 	LATB,CKnum,0
					BCF 	STATUS,0,0		//Carry
					BTFSC 	PORTB,Dnum,0
					BSF 	STATUS,0,0		//Carry
					RRCF	d,1,0
					decfsz	i,1,0
					BRA 	ciclo_r194
					_endasm
					D0();
					INTCONbits.GIE=1;
					TXins(LOBYTE(d));
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case I2C_INIT:				//I2C init, 0xff = off, slew (1b), bit rate (3b), A2-A1-A0 (3b)
				TXins(I2C_INIT);
				if(RXptr+1<number_of_bytes_read){
					i=receive_buffer[++RXptr];
#if !defined(SW_I2C)					//hardware peripheral
					if (i==0xff){
						SSPCON1=0;              // disable synchronous serial port
						TRISB=0xff;
						WP_dir=1;				//WP
					}
					else{
						SSPSTAT = 0x80;
						SSPCON1 = 0x08;			//master mode
						SSPCON2 = 0x00;
						if(i&0x40) SSPSTATbits.SMP=0;	//slew rate on
						TRISB=I2C_mask;			//Ax
						LATB=0;
						if(i&1)	A0=1;
						if(i&2)	A1=1;
						if(i&4)	A2=1;
						WP_dir=0;				//WP
						WP=0;
						//F=Fosc/4/(SPADD+1)
						i&=0x38;				// 0011 1000
						if(i==0)			SSPADD=119;		//100k
						else if(i==0x8)		SSPADD=59;		//200k
						else if(i==0x10)	SSPADD=39;		//400k
						else if(i==0x18)	SSPADD=14;		//800k
						else if(i==0x20)	SSPADD=11;		//1M
						SSPCON1bits.SSPEN=1;              // enable synchronous serial port
					}
#else					//software mode
					if (i==0xff){
						TRISB=0xff;
						WP_dir=1;				//WP
					}
					else{
						TRISB=I2C_mask;			//Ax
						LATB=0;
						if(i&1)	A0=1;
						if(i&2)	A1=1;
						if(i&4)	A2=1;
						WP_dir=0;				//WP
						WP=0;
						T2=5;
					}
#endif
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case I2C_READ:				//I2C read (nbytes, 2B address) 	n=0 -> current address read
				TXins(I2C_READ);
#if !defined(SW_I2C)					//hardware peripheral
				if(RXptr+3<number_of_bytes_read){
					i=receive_buffer[++RXptr];						//save nbytes
					HIBYTE(d)=1;
					LOBYTE(d)=receive_buffer[++RXptr];				//save address1
					for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
					for(SSPCON2bits.SEN=1;SSPCON2bits.SEN;);		//start
					if(i){
						for(SSPBUF=LOBYTE(d)&0xFE;SSPSTATbits.BF;);		//address1 write
						for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
						if(!SSPCON2bits.ACKSTAT){						//acknowledge ?
							for(SSPBUF=receive_buffer[++RXptr];SSPSTATbits.BF;);	//address2
							for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
							if(!SSPCON2bits.ACKSTAT){						//acknowledge ?
								for(SSPCON2bits.RSEN=1;SSPCON2bits.RSEN;);		//restart
								for(SSPBUF=LOBYTE(d)|0x01;SSPSTATbits.BF;);		//address1 read
								for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
								if(!SSPCON2bits.ACKSTAT){						//acknowledge ?
									HIBYTE(d)=0;
									for(TXins(i);i;i--){						//bytes requested
										for(SSPCON2bits.RCEN=1;!SSPSTATbits.BF;);	//receive byte
										TXins(SSPBUF);								//write to queue
										if(i>1)	for(SSPCON2bits.ACKDT=0,SSPCON2bits.ACKEN=1;SSPCON2bits.ACKEN;); //acknowledge
									}
									for(SSPCON2bits.ACKDT=1,SSPCON2bits.ACKEN=1;SSPCON2bits.ACKEN;); //not acknowledge
								}
							}
						}
					}
					else{
						for(SSPBUF=LOBYTE(d)|0x01;SSPSTATbits.BF;);		//address1 read
						for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
						if(!SSPCON2bits.ACKSTAT){						//acknowledge ?
							HIBYTE(d)=0;
							TXins(1);
							for(SSPCON2bits.RCEN=1;!SSPSTATbits.BF;);	//receive byte
							TXins(SSPBUF);								//write to queue
							for(SSPCON2bits.ACKDT=1,SSPCON2bits.ACKEN=1;SSPCON2bits.ACKEN;); //not acknowledge
						}
					}
					if(SSPCON2bits.ACKSTAT&&HIBYTE(d)){
						TXins(ACK_ERR);
						receive_buffer[RXptr+1]=FLUSH;
					}
					for(SSPCON2bits.PEN=1;SSPCON2bits.PEN;);	//stop
				}
#else				//software mode
				if(RXptr+3<number_of_bytes_read){
					i=receive_buffer[++RXptr];						//save nbytes
					HIBYTE(d)=1;
					LOBYTE(d)=receive_buffer[++RXptr];			//save address1
					SWStartI2C();
					if(i){
						SWPutcI2C(LOBYTE(d)&0xFE);		//address1 write
						if(!SWAckI2C()){						//acknowledge ?
							SWPutcI2C(receive_buffer[++RXptr]);	//address2
							if(!SWAckI2C()){						//acknowledge ?
								SWRestartI2C();
								SWPutcI2C(LOBYTE(d)|0x01);		//address1 read
								if(!SWAckI2C()){						//acknowledge ?
									HIBYTE(d)=0;
									for(TXins(i);i;i--){						//bytes requested
										TXins(SWGetcI2C());								//write to queue
										if(i>1)	AckI2C(); //acknowledge
									}
									NAckI2C(); //not acknowledge
								}
							}
						}
					}
					else{
						SWPutcI2C(LOBYTE(d)|0x01);		//address1 read
						if(!SWAckI2C()){						//acknowledge ?
							TXins(1);
							TXins(SWGetcI2C());	//receive byte
							NAckI2C(); //not acknowledge
						}
					}
					if(SWAckI2C()&&HIBYTE(d)){
						TXins(ACK_ERR);
						receive_buffer[RXptr+1]=FLUSH;
					}
					SWStopI2C();	//stop
				}
#endif
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case I2C_READ2:				//I2C read 2 (nbytes, 3B address)
				TXins(I2C_READ2);
#if !defined(SW_I2C)					//hardware peripheral
				if(RXptr+4<number_of_bytes_read){
					i=receive_buffer[++RXptr];						//save nbytes
					HIBYTE(d)=1;
					LOBYTE(d)=receive_buffer[++RXptr];				//save address1
					for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
					for(SSPCON2bits.SEN=1;SSPCON2bits.SEN;);		//start
					for(SSPBUF=LOBYTE(d)&0xFE;SSPSTATbits.BF;);		//address1 write
					for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
					if(!SSPCON2bits.ACKSTAT){						//acknowledge ?
						for(SSPBUF=receive_buffer[++RXptr];SSPSTATbits.BF;);	//address2
						for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
						if(!SSPCON2bits.ACKSTAT){						//acknowledge ?
							for(SSPBUF=receive_buffer[++RXptr];SSPSTATbits.BF;);	//address3
							for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
							if(!SSPCON2bits.ACKSTAT){						//acknowledge ?
								for(SSPCON2bits.RSEN=1;SSPCON2bits.RSEN;);		//restart
								for(SSPBUF=LOBYTE(d)|0x01;SSPSTATbits.BF;);		//address1 read
								for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););		//wait idle
								if(!SSPCON2bits.ACKSTAT){						//acknowledge ?
									HIBYTE(d)=0;
									for(TXins(i);i;i--){						//bytes requested
										for(SSPCON2bits.RCEN=1;!SSPSTATbits.BF;);	//receive byte
										TXins(SSPBUF);								//write to queue
										if(i>1)	for(SSPCON2bits.ACKDT=0,SSPCON2bits.ACKEN=1;SSPCON2bits.ACKEN;); //acknowledge
									}
									for(SSPCON2bits.ACKDT=1,SSPCON2bits.ACKEN=1;SSPCON2bits.ACKEN;); //not acknowledge
								}
							}
						}
					}
					if(SSPCON2bits.ACKSTAT&&HIBYTE(d)){
						TXins(ACK_ERR);
						receive_buffer[RXptr+1]=FLUSH;
					}
					for(SSPCON2bits.PEN=1;SSPCON2bits.PEN;);	//stop
				}
#else				//software mode
				if(RXptr+4<number_of_bytes_read){
					i=receive_buffer[++RXptr];						//save nbytes
					HIBYTE(d)=1;
					LOBYTE(d)=receive_buffer[++RXptr];			//save address1
					SWStartI2C();
					SWPutcI2C(LOBYTE(d)&0xFE);		//address1 write
					if(!SWAckI2C()){						//acknowledge ?
						SWPutcI2C(receive_buffer[++RXptr]);	//address2
						if(!SWAckI2C()){						//acknowledge ?
							SWPutcI2C(receive_buffer[++RXptr]);	//address3
							if(!SWAckI2C()){						//acknowledge ?
								SWRestartI2C();
								SWPutcI2C(LOBYTE(d)|0x01);		//address1 read
								if(!SWAckI2C()){						//acknowledge ?
									HIBYTE(d)=0;
									for(TXins(i);i;i--){						//bytes requested
										TXins(SWGetcI2C());								//write to queue
										if(i>1)	AckI2C(); //acknowledge
									}
									NAckI2C(); //not acknowledge
								}
							}
						}
					}
					if(SWAckI2C()&&HIBYTE(d)){
						TXins(ACK_ERR);
						receive_buffer[RXptr+1]=FLUSH;
					}
					SWStopI2C();	//stop
				}
#endif
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case I2C_WRITE:				//I2C write (nbytes, 2B address)
				TXins(I2C_WRITE);
				i=receive_buffer[++RXptr];
#if !defined(SW_I2C)					//hardware peripheral
				if(RXptr+2+i<number_of_bytes_read){
					for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););			//wait idle
					for(SSPCON2bits.SEN=1;SSPCON2bits.SEN;);			//start
					for(SSPBUF=receive_buffer[++RXptr]&0xFE;SSPSTATbits.BF;);		//address1 write
					for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););			//wait idle
					if(!SSPCON2bits.ACKSTAT){							//acknowledge ?
						for(SSPBUF=receive_buffer[++RXptr];SSPSTATbits.BF;);	//address2
						for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););				//wait idle
						if(!SSPCON2bits.ACKSTAT){								//acknowledge ?
							TXins(i);
							for(;i&&!SSPCON2bits.ACKSTAT;i--){						//bytes requested
								for(SSPBUF=receive_buffer[++RXptr];SSPSTATbits.BF;);	//byte write
								for(;(SSPCON2&0x1F)|(SSPSTATbits.R_W););				//wait idle
							}
						}
					}
					if(i||SSPCON2bits.ACKSTAT){
						TXins(ACK_ERR);
						receive_buffer[RXptr+1]=FLUSH;
					}
					for(SSPCON2bits.PEN=1;SSPCON2bits.PEN;);	//stop
				}
#else
				if(RXptr+2+i<number_of_bytes_read){
					SWStartI2C();
					SWPutcI2C(receive_buffer[++RXptr]&0xFE);		//address1 write
					if(!SWAckI2C()){						//acknowledge ?
						SWPutcI2C(receive_buffer[++RXptr]);	//address2
						if(!SWAckI2C()){						//acknowledge ?
							TXins(i);
							for(;i;i--){						//bytes requested
								SWPutcI2C(receive_buffer[++RXptr]);	//byte write
								if(SWAckI2C()){
									i=0xff;
									break;
								}
							}
						}
					}
					if(i){
						TXins(ACK_ERR);
						receive_buffer[RXptr+1]=FLUSH;
					}
					SWStopI2C();	//stop
				}
#endif
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SPI_INIT:				//SPI init, 0xff = off, smp(1b), mode (2b), bit rate (2b)
				TXins(SPI_INIT);
				if(RXptr+1<number_of_bytes_read){
					i=receive_buffer[++RXptr];
#if !defined(SW_SPI)					//hardware peripheral
					if (i==0xff){
						SSPCON1=0;              // disable synchronous serial port
						TRISB=0xff;
						WP_dir=1;				//WP
						SDO_dir=1;				//SDO
					}
					else{
						SSPSTAT = 0;
						SSPCON1 = 0x03;			//timer 2 source
						if(i&0x08) SSPCON1bits.CKP=1;		//CKP
						if((i&0x04)==0) SSPSTATbits.CKE=1;	//CKE
						if(i&0x10) SSPSTATbits.SMP=1;		//SMP
						//SPI_CS=1;
						//SPI_HLD=1;
						TRISB=SPI_mask;
						WP_dir=0;					//WP=RESET
						//WP=0;
						SDO_dir=0;					//SDO
						i&=0x03;					// 00000011
						//Fck=Fosc/8/(PR2+1)
						if(i==0) PR2=0x3b;			//100 kbps
						else if(i==1) PR2=0x1d;		//200 kbps
						else if(i==2) PR2=0x13;		//300 kbps
						else if(i==3) PR2=0xb;		//500 kbps
						T2CON = 4;					//timer2 ON
						SSPCON1bits.SSPEN=1;		// enable synchronous serial port
					}
#else					//software mode
					if (i==0xff){
						TRISB=0xff;
						WP_dir=1;				//WP
						SDO_dir=1;				//SDO
						T1=1;
						T2=100;
					}
					else{
						TRISB=SPI_mask;
						if(i&8) SPICK=1;		//clock polarity
						else SPICK=0;
						WP_dir=0;					//WP=RESET
						WP=0;
						SDO_dir=0;					//SDO
						T2=i&0xC;				//mode
						i&=0x03;					// 00000011
						if(i==0) T1=18;			//100 kbps
						else if(i==1) T1=8;		//200 kbps
						else if(i==2) T1=5;		//300 kbps
						else if(i==3) T1=2;		//500 kbps
					}
#endif
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SPI_READ:				//SPI read (nbytes) if n=0 reads SSPBF
				TXins(SPI_READ);
				if(RXptr+1<number_of_bytes_read){
					i=receive_buffer[++RXptr];				//save nbytes
					TXins(i);
#if !defined(SW_SPI)					//hardware peripheral
					if(!i) TXins(SSPBUF);					//if n=0 reads SSPBF
					else{
						TRISBbits.TRISB0=1;		//D input
						for(;i;i--){						//bytes requested
							PIR1bits.SSPIF=0;
							i2=SSPBUF;						//workaround for rev3 errata
							for(SSPBUF=0;!PIR1bits.SSPIF;);		//read
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							TXins(SSPBUF);							//write to queue
						}
					}
#else					//software mode
					if(!i) TXins(0);					//if n=0 reads 0
					else{
						TRISBbits.TRISB0=1;		//D input
						for(;i;i--){						//bytes requested
							TXins(SW_IO_SPI(0));			//write to queue
						}
					}
#endif
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SPI_WRITE:				//SPI write (nbytes)
				TXins(SPI_WRITE);
				i=receive_buffer[++RXptr];
				if(RXptr+i<number_of_bytes_read){
					TXins(i);
					TRISCbits.TRISC7=0;		//D output
					for(;i;i--){						//bytes requested
#if !defined(SW_SPI)					//hardware peripheral
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=receive_buffer[++RXptr];!PIR1bits.SSPIF;);		//write
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
#else					//software mode
						SW_IO_SPI(receive_buffer[++RXptr]);
#endif
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case EXT_PORT:				//EXT_PORT, controls expansion lines on PORTB,A,C
				TXins(EXT_PORT);
				if(RXptr+2<number_of_bytes_read){
					LATB=receive_buffer[++RXptr];
					i=receive_buffer[++RXptr];
					LATC=(LATC&0x3F)|(i&0xC0); 	//00111111
					LATA=(LATA&0xC7)|(i&0x38); 	//11000111
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case AT_READ_DATA:				//read n words from Atmel via SPI: 0x20 + addr
				TXins(AT_READ_DATA);
				if(RXptr+3<number_of_bytes_read){
					i=receive_buffer[++RXptr];				//save nword
					HIBYTE(d)=receive_buffer[++RXptr];		//save address
					LOBYTE(d)=receive_buffer[++RXptr];
					if(TXptr+i+i<HID_INPUT_REPORT_BYTES){
						TXins(i);
						for(;i;i--){
#if !defined(SW_SPI)					//hardware peripheral
							PIR1bits.SSPIF=0;
							i2=SSPBUF;						//workaround for rev3 errata
							for(SSPBUF=0x20;!PIR1bits.SSPIF;);		//0x20
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							PIR1bits.SSPIF=0;
							i2=SSPBUF;						//workaround for rev3 errata
							for(SSPBUF=HIBYTE(d);!PIR1bits.SSPIF;);	//address H
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							PIR1bits.SSPIF=0;
							i2=SSPBUF;						//workaround for rev3 errata
							for(SSPBUF=LOBYTE(d);!PIR1bits.SSPIF;);	//address L
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							PIR1bits.SSPIF=0;
							i2=SSPBUF;						//workaround for rev3 errata
							for(SSPBUF=0;!PIR1bits.SSPIF;);			//read
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							TXins(SSPBUF);							//write to queue
							PIR1bits.SSPIF=0;
							for(SSPBUF=0x28;!PIR1bits.SSPIF;);		//0x28
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							PIR1bits.SSPIF=0;
							i2=SSPBUF;						//workaround for rev3 errata
							for(SSPBUF=HIBYTE(d);!PIR1bits.SSPIF;);	//address H
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							PIR1bits.SSPIF=0;
							i2=SSPBUF;						//workaround for rev3 errata
							for(SSPBUF=LOBYTE(d);!PIR1bits.SSPIF;);	//address L
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							PIR1bits.SSPIF=0;
							i2=SSPBUF;						//workaround for rev3 errata
							for(SSPBUF=0;!PIR1bits.SSPIF;);			//read
							for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
							TXins(SSPBUF);							//write to queue/**/
#else					//software mode
							SW_IO_SPI(0x20);		//0x20
							SW_IO_SPI(HIBYTE(d));	//address H
							SW_IO_SPI(LOBYTE(d));	//address L
							TXins(SW_IO_SPI(0));						//write to queue
							SW_IO_SPI(0x28);		//0x28
							SW_IO_SPI(HIBYTE(d));	//address H
							SW_IO_SPI(LOBYTE(d));	//address L
							TXins(SW_IO_SPI(0));						//write to queue
#endif
							d++;
						}
					}
					else{
						TXins(RX_ERR);
						receive_buffer[RXptr+1]=FLUSH;
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case AT_LOAD_DATA:				//write n words from Atmel via SPI: 0x20 + addr
				TXins(AT_LOAD_DATA);
				i=receive_buffer[++RXptr];				//save nword
				if(RXptr+i+i+2<number_of_bytes_read){
					TXins(i);
					HIBYTE(d)=receive_buffer[++RXptr];		//save address
					LOBYTE(d)=receive_buffer[++RXptr];
					for(;i;i--){
#if !defined(SW_SPI)					//hardware peripheral
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=0x40;!PIR1bits.SSPIF;);			//0x40
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=HIBYTE(d);!PIR1bits.SSPIF;);		//address H
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=LOBYTE(d);!PIR1bits.SSPIF;);		//address L
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=receive_buffer[++RXptr];!PIR1bits.SSPIF;); //data L
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=0x48;!PIR1bits.SSPIF;);			//0x48
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=HIBYTE(d);!PIR1bits.SSPIF;);		//address H
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=LOBYTE(d);!PIR1bits.SSPIF;);		//address L
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
						PIR1bits.SSPIF=0;
						i2=SSPBUF;						//workaround for rev3 errata
						for(SSPBUF=receive_buffer[++RXptr];!PIR1bits.SSPIF;); //data H
						for(i2=PR2>>1;i2;i2--);			//workaround for rev3 errata
#else					//software mode
						SW_IO_SPI(0x40);		//0x40
						SW_IO_SPI(HIBYTE(d));	//address H
						SW_IO_SPI(LOBYTE(d));	//address L
						SW_IO_SPI(receive_buffer[++RXptr]);	//data L
						SW_IO_SPI(0x48);		//0x48
						SW_IO_SPI(HIBYTE(d));	//address H
						SW_IO_SPI(LOBYTE(d));	//address L
						SW_IO_SPI(receive_buffer[++RXptr]);	//data H
#endif
						d++;
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case CLOCK_GEN:				//
				TXins(CLOCK_GEN);
				if(RXptr+1<number_of_bytes_read){
					i=receive_buffer[++RXptr];
#if !defined(NO_CCP2)				//use CCP1-1-timer3
					if (i==0xff){
						CCP1CON=CCP2CON=0;
						LATBbits.LATB3=0;
						T3CON=0;
					}
					else{
						TRISCbits.TRISC2=1;			//PWM1 disable output
						TRISBbits.TRISB3=0;			//PWM2 enable output
						TMR3H=0xFF;					//precharge near full scale
						TMR3L=0;
						T3CON=0b11000001;	//16 bit, source for CCP1 & CCP2, no prescaler
						CCPR1H=CCPR2H=0x00;
						//CCPR1=N clock=12MHz/2(N+1)
						if(i==0) CCPR1L=0x3B;		//100KHz
						else if(i==1) CCPR1L=0x1D;	//200KHz
						else if(i==2) CCPR1L=0xB;	//500KHz
						else if(i==3) CCPR1L=0x5;	//1 MHz
						else if(i==4) CCPR1L=0x2;	//2 MHz
						else if(i==5) CCPR1L=0x1;	//3 MHz
						else if(i==6) CCPR1L=0x0;	//6 MHz
						CCPR2L=1;
						CCP1CON=0x0B;		//reset timer3 on match
						CCP2CON=0x02;		//toggle on timer3 match
					}
#else					//18F2450 (software mode)
					if (i==0xff){
						LATBbits.LATB3=0;
					}
					else{
						TRISBbits.TRISB3=0;
						LATBbits.LATB3=1;
					}
#endif
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SIX:				//Core instruction (PIC24) 0000, 24 bit data [41us]
				TXins(SIX);
				if(RXptr+3<number_of_bytes_read){
					INTCONbits.GIE=0;
					Ddir_bit=0;		//Output
					D0();
					CKpulseL();
					CKpulseL();
					CKpulseL();
					CKpulseL();
					HIBYTE(d)=receive_buffer[++RXptr];	//H
					LOBYTE(d)=receive_buffer[++RXptr];	//
					i2=receive_buffer[++RXptr];			//L
					_asm
					MOVLW	24
					movwf	i,0
					ciclo_r195:
					RRCF	d+1,1,0
					RRCF	d,1,0
					RRCF	i2,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					nop
					BSF 	LATB,CKnum,0	//CKpulse();
					nop
					nop
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r195
					_endasm
					D0();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SIX_LONG:				//Core instruction (PIC24) 0000, 24 bit data + 2 NOP [97us]
				TXins(SIX_LONG);
				if(RXptr+3<number_of_bytes_read){
					INTCONbits.GIE=0;
					Ddir_bit=0;		//Output
					D0();
					CKpulseL();
					CKpulseL();
					CKpulseL();
					CKpulseL();
					HIBYTE(d)=receive_buffer[++RXptr];	//H
					LOBYTE(d)=receive_buffer[++RXptr];	//
					i2=receive_buffer[++RXptr];			//L
					_asm
					MOVLW	24
					movwf	i,0
					ciclo_r196:
					RRCF	d+1,1,0
					RRCF	d,1,0
					RRCF	i2,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					nop
					BSF 	LATB,CKnum,0	//CKpulse();
					nop
					nop
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r196
					_endasm
					D0();
					for(i=0;i<56;i++)CKpulseL();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SIX_LONG5:				//Core instruction (PIC24) 0000, 24 bit data + 5 NOP [275us]
				TXins(SIX_LONG5);
				if(RXptr+3<number_of_bytes_read){
					INTCONbits.GIE=0;
					Ddir_bit=0;		//Output
					D0();
					CKpulseL();
					CKpulseL();
					CKpulseL();
					CKpulseL();
					HIBYTE(d)=receive_buffer[++RXptr];	//H
					LOBYTE(d)=receive_buffer[++RXptr];	//
					i2=receive_buffer[++RXptr];			//L
					_asm
					MOVLW	24
					movwf	i,0
					ciclo_r196b:
					RRCF	d+1,1,0
					RRCF	d,1,0
					RRCF	i2,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					nop
					BSF 	LATB,CKnum,0	//CKpulse();
					nop
					nop
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA 	ciclo_r196b
					_endasm
					D0();
					for(i=0;i<140;i++)CKpulseL();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			//Core instruction (PIC24) 0000, 24 bit data * N
			//N[7:6] = extra NOP after each SIX
			//[(7+38*N(5:0)+28*N(7:6)*N(5:0))us]
			case SIX_N:
				TXins(SIX_N);
				i=receive_buffer[++RXptr];
				i2=i&0x3F;
				if(RXptr+i2+i2+i2<number_of_bytes_read){
					byte j,n;
					n=i&0xC0;
					if(n==0x40) n=28;
					else if(n==0x80) n=56;
					else if(n==0xC0) n=84;
					INTCONbits.GIE=0;
					Ddir_bit=0;		//Output
					for(j=i&0x3F;j;j--){
						D0();
						CKpulseL();
						CKpulseL();
						CKpulseL();
						CKpulseL();
						HIBYTE(d)=receive_buffer[++RXptr];	//H
						LOBYTE(d)=receive_buffer[++RXptr];	//
						i2=receive_buffer[++RXptr];			//L
						_asm
						MOVLW	24
						movwf	i,0
						ciclo_r195b:
						RRCF	d+1,1,0
						RRCF	d,1,0
						RRCF	i2,1,0
						BTFSS 	STATUS,0,0		//Carry
						BCF 	LATB,Dnum,0	//D0();
						BTFSC 	STATUS,0,0		//Carry
						BSF 	LATB,Dnum,0	//D1();
						nop
						BSF 	LATB,CKnum,0	//CKpulse();
						nop
						nop
						nop
						BCF 	LATB,CKnum,0
						decfsz	i,1,0
						BRA 	ciclo_r195b
						_endasm
						D0();
						for(i=n;i;i--)CKpulseL();
					}
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case REGOUT:				//Read out (PIC24) 1000, 16 bit data [38us]
				TXins(REGOUT);
				INTCONbits.GIE=0;
				Ddir_bit=0;		//Output
				D1();
				CKpulseL();
				D0();
				for(i=0;i<11;i++)CKpulseL();
				Ddir_bit=1;		//Input
				_asm
				MOVLW	16
				movwf	i,0
				ciclo_r197:
				BSF 	LATB,CKnum,0	//CKpulse();
				nop
				nop
				nop
				BCF 	STATUS,0,0		//Carry
				BTFSC 	PORTB,Dnum,0
				BSF 	STATUS,0,0		//Carry
				RRCF	d+1,1,0
				RRCF	d,1,0
				BCF 	LATB,CKnum,0
				decfsz	i,1,0
				BRA 	ciclo_r197
				_endasm
				D0();
				INTCONbits.GIE=1;
				TXins(HIBYTE(d));
				TXins(LOBYTE(d));
				break;
			case ICSP_NOP:				//NOP Core instruction (PIC24) 0000, 24 bit data [31.5us]
				TXins(ICSP_NOP);
				INTCONbits.GIE=0;
				Ddir_bit=0;		//Output
				D0();
				for(i=0;i<28;i++)CKpulseL();
				INTCONbits.GIE=1;
				break;
			case TX16:
			//Transmit N*16 bit data MSB first
			//Clock period ~ (2*(T1-1)+1.8)us
			//Execution time: N*(31.5us+(T1-1)*32us)
				TXins(TX16);
				i=receive_buffer[++RXptr];
				if(RXptr+i+i<number_of_bytes_read){
					byte j;
					INTCONbits.GIE=0;
					Ddir_bit=0;		//Output
					for(j=i;j;j--){
						HIBYTE(d)=receive_buffer[++RXptr];	//H
						LOBYTE(d)=receive_buffer[++RXptr];	//L
						_asm
						MOVLW	16
						movwf	i,0
						ciclo_r198:
						RLCF	d,1,0
						RLCF	d+1,1,0
						BTFSS 	STATUS,0,0		//Carry
						BCF 	LATB,Dnum,0	//D0();
						BTFSC 	STATUS,0,0		//Carry
						BSF 	LATB,Dnum,0	//D1();
						nop
						BSF 	LATB,CKnum,0	//CKpulse();
						movff	T1,i2
					ciclo_r198_t1:
						decf	i2,1,0
						bz		ciclo_r198_t2
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						BRA 	ciclo_r198_t1
					ciclo_r198_t2:
						BCF 	LATB,CKnum,0
						movff	T1,i2
					ciclo_r198_t3:
						decf	i2,1,0
						bz		ciclo_r198_t4
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						BRA ciclo_r198_t3
					ciclo_r198_t4:
						decfsz	i,1,0
						BRA 	ciclo_r198
						_endasm
					}
					D0();
					Ddir_bit=1;		//Input
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case RX16:				//Read N*16 bit data MSB first
				//Clock period ~ (2*(T1-1)+1.8)us
				//Execution time: N*(31.5us+(T1-1)*32us)
				TXins(RX16);
				i=receive_buffer[++RXptr];
				if(RXptr<number_of_bytes_read&&TXptr+i+i<HID_INPUT_REPORT_BYTES){
					byte j;
					TXins(i);
					INTCONbits.GIE=0;
					Ddir_bit=1;		//Input
					for(j=i;j;j--){
						_asm
						MOVLW	16
						movwf	i,0
						ciclo_r199:
						BSF 	LATB,CKnum,0	//CKpulse();
						movff	T1,i2
					ciclo_r199_t1:
						decf	i2,1,0
						bz		ciclo_r199_t2
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						BRA 	ciclo_r199_t1
					ciclo_r199_t2:
						BCF 	STATUS,0,0		//Carry
						BTFSC 	PORTB,Dnum,0
						BSF 	STATUS,0,0		//Carry
						RLCF	d,1,0
						RLCF	d+1,1,0
						BCF 	LATB,CKnum,0
						movff	T1,i2
					ciclo_r199_t3:
						decf	i2,1,0
						bz		ciclo_r199_t4
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						BRA ciclo_r199_t3
					ciclo_r199_t4:
						decfsz	i,1,0
						BRA 	ciclo_r199
						_endasm
						TXins(HIBYTE(d));
						TXins(LOBYTE(d));
					}
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case uW_INIT:				//microwire init
				TXins(uW_INIT);
				TRISB=0xC5;				//11000101
				LATB=0;
				uWDO_dir=0;
				break;
			case uWTX:				//Transmit N bit data
				TXins(uWTX);
				i=receive_buffer[++RXptr];
				if(RXptr+(i-1)>>3+1<number_of_bytes_read){
					INTCONbits.GIE=0;
					i2=0;
					for(;i>0;i--,i2--){
						if(i2==0){
							LOBYTE(d)=receive_buffer[++RXptr];
							i2=8;
						}
						_asm
						RLCF	d,1,0
						BTFSS 	STATUS,0,0		//Carry
						BCF 	LATC,uWDOnum,0	//D0();
						BTFSC 	STATUS,0,0		//Carry
						BSF 	LATC,uWDOnum,0	//D1();
						BSF 	LATB,uWCKnum,0	//CKpulse();
						nop
						nop
						nop
						nop
						BCF 	LATB,uWCKnum,0
						_endasm
					}
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case uWRX:				//Receive N bit data
				TXins(uWRX);
				if(RXptr+1<number_of_bytes_read&&TXptr+(i-1)>>3+1<HID_INPUT_REPORT_BYTES){
					i=receive_buffer[++RXptr];
					TXins(i);
					INTCONbits.GIE=0;
					LOBYTE(d)=0;
					i2=8;
					for(;i>0;i--){
						_asm
						BSF 	LATB,uWCKnum,0	//CKpulse();
						nop
						nop
						nop
						nop
						BCF 	LATB,uWCKnum,0
						BCF 	STATUS,0,0		//Carry
						BTFSC 	PORTB,uWDInum,0
						BSF 	STATUS,0,0		//Carry
						RLCF	d,1,0
						_endasm
						i2--;
						if(i2==0||i==1){
							i2=8;
							TXins(LOBYTE(d));
							LOBYTE(d)=0;
						}
					}
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case OW_RESET:		//One-wire reset pulse
				//Returns 1 if presence pulse is detected, otherwise 0
				//Execution time: ~880us
				TXins(OW_RESET);
				INTCONbits.GIE=0;
				SDA=1;
				SDA_dir=0;
				Nop();
				SDA=0;
				Delay1us(250);	//250us
				Delay1us(250);	//250us
				SDA=1;		//active pull-up
				Nop();
				SDA_dir=1;	//hiZ
				Delay1us(70);	//70us
				if(SDA_p==0) TXins(1);	//1 if presence pulse
				else TXins(0);
				Delay1us(150);	//300us
				Delay1us(150);	//300us
				INTCONbits.GIE=1;
				break;
			case OW_WRITE:		//One-wire write N bytes (LSB first)
				//Parameters: N + N data bytes
				//Returns RX_ERR if N is too high
				//Uses i,i2
				//Execution time: 6us + N*576us
				TXins(OW_WRITE);
				i=receive_buffer[++RXptr];
				if(RXptr+i<number_of_bytes_read){
					byte k;
					for(;i;i--){
						i2=receive_buffer[++RXptr];
						INTCONbits.GIE=0;
						for(k=8;k>0;k--){
							SDA=0;
							SDA_dir=0;		//start pulse
							if(i2&1){		//write 1
								Delay1us(4);	//4us
								SDA=1;		//active pull-up
								//Nop();
								//SDA_dir=1;	//hiZ
								Delay1us(66);	//66us
							}
							else{			//write 0
								Delay1us(60);	//60us
								SDA=1;		//active pull-up
								//Nop();
								//SDA_dir=1;	//hiZ
								Delay1us(10);	//10us
							}
							i2=i2>>1;
						}
						INTCONbits.GIE=1;
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case OW_READ:		//One-wire read N bytes (LSB first)
				//Parameters: N
				//Returns RX_ERR if N is too high
				//Uses i,i2
				//Execution time: 20us + N*582us
				TXins(OW_READ);
				i=receive_buffer[++RXptr];
				if(RXptr<number_of_bytes_read&&TXptr+i<HID_INPUT_REPORT_BYTES){
					byte k;
					TXins(i);
					for(;i;i--){
						INTCONbits.GIE=0;
						i2=0;
						for(k=8;k>0;k--){
							SDA=0;
							SDA_dir=0;		//start pulse
							Delay1us(6);	//6us
							SDA=1;		//active pull-up
							Nop();
							SDA_dir=1;	//hiZ
							Delay1us(8);	//8us
							i2=i2>>1;
							if(SDA_p==1) i2+=0x80;
							Delay1us(40);	//40us
							SDA=1;		//active pull-up
							SDA_dir=0;
							Nop();
							SDA_dir=1;	//hiZ
							Delay1us(16);	//16us
						}
						TXins(i2);
						INTCONbits.GIE=1;
					}
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case UNIO_STBY:		// UNI/O standby pulse
				//Execution time: ~621us
				TXins(UNIO_STBY);
				INTCONbits.GIE=0;
				SDA=0;
				SDA_dir=0;
				Delay1us(10);	//10us
				SDA=1;
				Delay1us(250);	//250us
				Delay1us(250);	//250us
				Delay1us(110);	//110us
				SDA_dir=1;	//hiZ
				INTCONbits.GIE=1;
				break;
			case UNIO_COM:		//UNI/O communication cycle (write+read)
				//Parameter1: N = bytes to write (MSB first)
				//Parameter2: M = bytes to read afer write
				//Parameter3: N bytes to write
				//Returns RX_ERR if N or M is too high
				//if N=0 no start header is transmitted
				//Returns M + M bytes
				//ACK_ERR is returned if slave does not acknowledge, then command ends
				//Uses i,i2
				//Execution time: 10us + (N+1+M)*210us
				TXins(UNIO_COM);
				i=receive_buffer[++RXptr];  //N
				i2=receive_buffer[++RXptr]; //M
				if(RXptr+i<number_of_bytes_read&&TXptr+i2<HID_INPUT_REPORT_BYTES){
					byte k,dx;
					INTCONbits.GIE=0;
					if(i){
						SDA=0;
						SDA_dir=0;		//start pulse
						Delay1us(20);	//20us
						SDA=1;
						Delay1us(10);	//10us
						SDA=0;
						Delay1us(20);	//20us
						SDA=1;
						Delay1us(20);	//20us
						SDA=0;
						Delay1us(20);	//20us
						SDA=1;
						Delay1us(20);	//20us
						SDA=0;
						Delay1us(20);	//20us
						SDA=1;
						Delay1us(20);	//20us
						SDA=0;
						Delay1us(20);	//20us
						SDA=1;
						Delay1us(10);	//10us
						SDA=0;			//MAK
						Delay1us(10);	//10us
						SDA=1;
						Delay1us(10);	//10us
						SDA_dir=1;		//HZ
						Delay1us(5);	//5us
						k=0;
						if(SDA_p==0) k=1;
						Delay1us(10);	//10us
						if(SDA_p==0) k=1;
						Delay1us(2);
						if(k){			//incorrect SAK
							TXins(ACK_ERR);
							i=i2=0;
							RXptr+=i;
						}
					}
					for(;i;i--){	//send N bytes (MSB first)
						dx=receive_buffer[++RXptr];
						SDA=0;
						SDA_dir=0;		//out
						for(k=8;k>0;k--){
							if(dx&0x80){		//write 1
								SDA=0;
								Delay1us(10);	//10us
								///Delay1us(5);	//5us
								SDA=1;
							}
							else{			//write 0
								SDA=1;
								Delay1us(10);	//10us
								SDA=0;
							}
							Delay1us(8);	//10us
							dx=dx<<1;
						}
						if(i2||i>1){
							SDA=0;			//MAK
							Delay1us(10);	//10us
							SDA=1;
						}
						else{
							SDA=1;			//NoMAK
							Delay1us(10);	//10us
							SDA=0;
						}
						Delay1us(10);	//10us
						SDA_dir=1;		//HZ
						Delay1us(5);	//5us
						k=0;
						//if(SDA_p==0) k=1; //Debug!!!
						if(SDA_p==1) k=1;
						Delay1us(10);	//10us
						if(SDA_p==0) k=1;
						if(k){			//incorrect SAK
							TXins(ACK_ERR);
							i=1;
							i2=0;
							RXptr+=i;
						}
						if(i>1){
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
						}
					}
					Nop();
					Nop();
					Nop();
					TXins(i2);
					for(;i2;i2--){	//receive N bytes (MSB first)
						dx=0;
						SDA_dir=1;		//input
						for(k=8;k>0;k--){
							Delay1us(3); // 1/4 bit
							dx=dx<<1;
							if(SDA_p==0){
								Delay1us(10);	//10us
								if(SDA_p==1) dx+=1;
								else dx+=0; //this is to keep constant timing
							}
							else{
								Delay1us(10);	//10us
								if(SDA_p==0) dx+=0;
								else dx+=0; //this is to keep constant timing
								Nop();
								Nop();
								Nop();
								Nop();
							}
							Nop();
							Nop();
							Nop();
							Nop();
							Nop();
							if(k>1) Delay1us(4);	//6us
							else Delay1us(2);
						}
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						SDA_dir=0;		//output
						if(i2>1){
							SDA=0;			//MAK
							Delay1us(10);	//10us
							SDA=1;
						}
						else{
							SDA=1;			//NoMAK
							Delay1us(10);	//10us
							SDA=0;
						}
						TXins(dx);
						Delay1us(7);	//10us
						SDA_dir=1;		//HZ
						Delay1us(5);	//5us
						k=0;
						//if(SDA_p==0) k=1; //Debug!!!
						if(SDA_p==1) k=1;
						Delay1us(10);	//10us
						if(SDA_p==0) k=1;
						Delay1us(3);	//3us
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						Nop();
						if(k){			//incorrect SAK
							TXins(ACK_ERR);
							i2=1;
						}
					}
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case SET_PORT_DIR:				//Sets the direction of expansion lines on PORTB,A,C
				TXins(SET_PORT_DIR);
				if(RXptr+2<number_of_bytes_read){
					TRISB=receive_buffer[++RXptr];
					i=receive_buffer[++RXptr];
					TRISC=(TRISC&0x3F)|(i&0xC0); 	//00111111
					TRISA=(TRISA&0xC7)|(i&0x38); 	//11000111
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case READ_B:				//read PORTB
				TXins(READ_B);
				TXins(PORTB);
				break;
			case READ_AC:				//read PORTA-C (RC7:RC6:RA5:RA4:RA3:0:0:0)
				TXins(READ_AC);
				TXins(PORTA&0x38|PORTC&0xC0);
				break;
			case AT_HV_RTX:
			//Transmit 2N*8 bit data MSB first on RB0/RC7 (PB1/PB0) CLK on RC6 (PB3)
			//byteN=PB1, byteN+1=PB0
			//Receive 8 bit data on RB1 (PB2) but return only the last byte read
			//Execution time: 5+N*18us
				TXins(AT_HV_RTX);
				i=receive_buffer[++RXptr];
				if(RXptr+i+i<number_of_bytes_read){
					byte j;
					INTCONbits.GIE=0;
					LATCbits.LATC6=0;		//PB3=0
					LATBbits.LATB0=0;		//PB1=0
					LATCbits.LATC7=0;		//PB0=0
					TRISBbits.TRISB0=0;		//Output
					TRISCbits.TRISC7=0;		//Output
					TRISCbits.TRISC6=0;		//Output
					for(j=i;j;j--){
						LOBYTE(d)=receive_buffer[++RXptr];	//PB1=RB0
						HIBYTE(d)=receive_buffer[++RXptr];	//PB0=RC7
						_asm
						BSF 	LATC,PB3,0 	//CKpulse();
						nop
						nop
						BCF 	LATC,PB3,0
						MOVLW	8
						movwf	i,0
						ciclo_r200:
						BCF 	STATUS,0,0		//Carry
						BTFSC 	PORTB,PB2,0
						BSF 	STATUS,0,0		//Carry
						RLCF	i2,1,0
						RLCF	d,1,0
						BTFSS 	STATUS,0,0		//Carry
						BCF 	LATB,PB1,0		//D0();
						BTFSC 	STATUS,0,0		//Carry
						BSF 	LATB,PB1,0 	//D1();
						RLCF	d+1,1,0
						BTFSS 	STATUS,0,0		//Carry
						BCF 	LATC,PB0,0		//D0();
						BTFSC 	STATUS,0,0		//Carry
						BSF 	LATC,PB0,0 	//D1();
						nop
						BSF 	LATC,PB3,0 	//CKpulse();
						nop
						nop
						BCF 	LATC,PB3,0
						decfsz	i,1,0
						BRA 	ciclo_r200
						BSF 	LATC,PB3,0 	//CKpulse();
						nop
						nop
						BCF 	LATC,PB3,0
						nop
						nop
						BSF 	LATC,PB3,0 	//CKpulse();
						nop
						nop
						BCF 	LATC,PB3,0
						BCF 	LATB,PB1,0		//D0();
						BCF 	LATC,PB0,0		//D0();
						_endasm
					}
					TXins(i2);
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case JTAG_SET_MODE:
			//Transmit 6 bit data MSB first
			//Execution time: 10us
				TXins(JTAG_SET_MODE);
				if(RXptr+1<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					_asm
					RLCF	d,1,0
					RLCF	d,1,0
					BCF 	LATB,TDInum,0	//D0();
					MOVLW	6
					movwf	i,0
					ciclo_201:
					RLCF	d,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,TMSnum,0	//TMS0;
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,TMSnum,0	//TMS1;
					BSF 	LATB,TCKnum,0	//TCK pulse;
					BCF 	LATB,TCKnum,0
                    decfsz	i,1,0
					BRA 	ciclo_201
					_endasm
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case JTAG_SEND_CMD:
			//Transmit 5 bit data LSB first
			//Execution time: 10us
				TXins(JTAG_SEND_CMD);
				if(RXptr+1<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					_asm
					BCF 	LATB,TDInum,0	//D0();
					BSF 	LATB,TMSnum,0	//TMS1;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TMSnum,0	//TMS0;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					MOVLW	4
					movwf	i,0
					ciclo_202:
					RRCF	d,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,TDInum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,TDInum,0	//D1();
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					decfsz	i,1,0
					BRA 	ciclo_202
					BSF 	LATB,TMSnum,0	//TMS1;
					RRCF	d,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,TDInum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,TDInum,0	//D1();
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TDInum,0	//D0();
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TMSnum,0	//TMS0;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					_endasm
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case JTAG_XFER_DATA:
			//Transfer 32 bit data LSB first
			//Execution time: 61us
				TXins(JTAG_XFER_DATA);
				if(RXptr+4<number_of_bytes_read){
					HIBYTE(dH)=receive_buffer[++RXptr];	//HH
					LOBYTE(dH)=receive_buffer[++RXptr];	//HL
					HIBYTE(d)=receive_buffer[++RXptr];	//H
					LOBYTE(d)=receive_buffer[++RXptr];	//L
					INTCONbits.GIE=0;
					_asm
					BCF 	LATB,TDInum,0	//D0();
					BSF 	LATB,TMSnum,0	//TMS1;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TMSnum,0	//TMS0;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					MOVLW	31
					movwf	i,0
					ciclo_203:
					RRCF	dH+1,1,0
					RRCF	dH,1,0
					RRCF	d+1,1,0
					RRCF	d,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,TDInum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,TDInum,0	//D1();
					BCF 	dH+1,7,0			//highest bit
					BTFSC 	PORTB,TDOnum,0
					BSF 	dH+1,7,0			//highest bit
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					decfsz	i,1,0
					BRA 	ciclo_203
					BSF 	LATB,TMSnum,0	//TMS1;
					RRCF	dH+1,1,0
					RRCF	dH,1,0
					RRCF	d+1,1,0
					RRCF	d,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,TDInum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,TDInum,0	//D1();
					BCF 	dH+1,7,0			//highest bit
					BTFSC 	PORTB,TDOnum,0
					BSF 	dH+1,7,0			//highest bit
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TDInum,0	//D0();
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TMSnum,0	//TMS0;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					_endasm
					TXins(HIBYTE(dH));
					TXins(LOBYTE(dH));
					TXins(HIBYTE(d));
					TXins(LOBYTE(d));
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case JTAG_XFER_F_DATA:
			//Transfer 32 bit data LSB first
			//Execution time: 61us
				TXins(JTAG_XFER_F_DATA);
				if(RXptr+4<number_of_bytes_read){
					HIBYTE(dH)=receive_buffer[++RXptr];	//HH
					LOBYTE(dH)=receive_buffer[++RXptr];	//HL
					HIBYTE(d)=receive_buffer[++RXptr];	//H
					LOBYTE(d)=receive_buffer[++RXptr];	//L
					INTCONbits.GIE=0;
					_asm
					BCF 	LATB,TDInum,0	//D0();
					BSF 	LATB,TMSnum,0	//TMS1;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TMSnum,0	//TMS0;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0
                    //PrAcc
					MOVLW	31
					movwf	i,0
					ciclo_204:
					RRCF	dH+1,1,0
					RRCF	dH,1,0
					RRCF	d+1,1,0
					RRCF	d,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,TDInum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,TDInum,0	//D1();
					BCF 	dH+1,7,0			//highest bit
					BTFSC 	PORTB,TDOnum,0
					BSF 	dH+1,7,0			//highest bit
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					decfsz	i,1,0
					BRA 	ciclo_204
					BSF 	LATB,TMSnum,0	//TMS1;
					RRCF	dH+1,1,0
					RRCF	dH,1,0
					RRCF	d+1,1,0
					RRCF	d,1,0
					BTFSS 	STATUS,0,0		//Carry
					BCF 	LATB,TDInum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,TDInum,0	//D1();
					BCF 	dH+1,7,0			//highest bit
					BTFSC 	PORTB,TDOnum,0
					BSF 	dH+1,7,0			//highest bit
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TDInum,0	//D0();
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					BCF 	LATB,TMSnum,0	//TMS0;
					BSF LATB,TCKnum,0//TCK pulse;
                    BCF LATB,TCKnum,0                    
					_endasm
					TXins(HIBYTE(dH));
					TXins(LOBYTE(dH));
					TXins(HIBYTE(d));
					TXins(LOBYTE(d));
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;

			//ICSP8 8-bit command without payload
			//parameter1: command code (MSB first)
			//execution time: 11us
			case ICSP8_SHORT:
				TXins(ICSP8_SHORT);
				if(RXptr+1<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					Ddir_bit=0;		//Output
					_asm
					MOVLW	8
					movwf	i,0
					ciclo_300:
					RLCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_300
					_endasm
					D0();
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;

			//ICSP8 8-bit command with 16-bit read
			//parameter1: command code (MSB first)
			//returns 2-bytes of data (MSB first)
			//execution time: 38us
			case ICSP8_READ:
				TXins(ICSP8_READ);
				if(RXptr+1<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					Ddir_bit=0;		//Output
					_asm
					MOVLW	8
					movwf	i,0
					ciclo_301:
					RLCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_301
					_endasm
					D0();
					Ddir_bit=1;		//Input
					for(i=0;i<2;i++) Nop();	//>1us
					for(i=0;i<7;i++) CKpulseN();	//ignore 7 bits (24-1-16)
                    d=0;
					_asm
					MOVLW	16
					movwf	i,0
					ciclo_302:
					BSF 	LATB,CKnum,0
					BCF 	STATUS,0,0		//Carry
					BCF 	LATB,CKnum,0
					BTFSC	PORTB,Dnum,0
					BSF 	STATUS,0,0		//Carry
					RLCF	d,1,0			//<<1 L
					RLCF	d+1,1,0			//<<1 H
					decfsz	i,1,0
					BRA ciclo_302
					_endasm
					CK1();	//Stop bit
					Ddir_bit=0;		//Output
					CK0();	//Stop bit
					INTCONbits.GIE=1;
					TXins(HIBYTE(d));	//&d+1
					TXins(LOBYTE(d));	//&d
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;

			//ICSP8 8-bit command with 16-bit payload
			//parameter1: command code (MSB first)
			//parameter2: 2-bytes of data (MSB first)
			//execution time: 35us
			case ICSP8_LOAD:
				TXins(ICSP8_LOAD);
				if(RXptr+3<number_of_bytes_read){
					LOBYTE(d)=receive_buffer[++RXptr];
					INTCONbits.GIE=0;
					Ddir_bit=0;		//Output
					_asm
					MOVLW	8
					movwf	i,0
					ciclo_303:
					RLCF	d,1,0
					BCF 	LATB,Dnum,0	//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0	//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_303
					_endasm
					D0();
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					for(i=0;i<7;i++) CKpulseN();	//ignore 7 bits (24-1-16)
					_asm
					MOVLW	16
					movwf	i,0
					ciclo_304:
					RLCF	d,1,0			//<<1 L
					RLCF	d+1,1,0			//<<1 H
					BCF 	LATB,Dnum,0		//D0();
					BTFSC 	STATUS,0,0		//Carry
					BSF 	LATB,Dnum,0		//D1();
					BSF 	LATB,CKnum,0	//CKpulseN();
					nop
					BCF 	LATB,CKnum,0
					decfsz	i,1,0
					BRA ciclo_304
					_endasm
					D0();
					CKpulseN();	//Stop bit
					INTCONbits.GIE=1;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;

/*****************************************/
// special instructions (debug)
			case LOOP:				//infinite loop
				for(;TXptr<HID_INPUT_REPORT_BYTES;TXptr++) transmit_buffer[TXptr]=0;
				TXptr=0;
   	   			if(!(ep1Bi.Stat&UOWN)) HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
   	 			RXptr=0xFF;
				break;
			case READ_RAM:			//read in RAM (2B addr)
				TXins(READ_RAM);
				if(RXptr+2<number_of_bytes_read){
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					TXins(HIBYTE(d));
					TXins(LOBYTE(d));
					FSR0H=HIBYTE(d);
					FSR0L=LOBYTE(d);
					i=INDF0;
					TXins(i);
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
			case WRITE_RAM:			//write to RAM (2B addr + data)
				TXins(WRITE_RAM);
				if(RXptr+3<number_of_bytes_read){
					HIBYTE(d)=receive_buffer[++RXptr];
					LOBYTE(d)=receive_buffer[++RXptr];
					i=receive_buffer[++RXptr];
					TXins(HIBYTE(d));
					TXins(LOBYTE(d));
					TXins(i);
					FSR0H=HIBYTE(d);
					FSR0L=LOBYTE(d);
					INDF0=i;
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
#if !defined(SW_SPI)					//hardware peripheral
			case SPI_TEST:				//SPI readback (nbytes)
				TXins(SPI_TEST);
				if(RXptr+3<number_of_bytes_read){
					unsigned int counter,errors;	//16 bit counter
					HIBYTE(counter)=receive_buffer[++RXptr];
					LOBYTE(counter)=receive_buffer[++RXptr];
					TRISCbits.TRISC7=0;		//D output
					errors=0;
					for(i=0;counter;counter--){						//bytes requested
						//PIR1bits.SSPIF=0;
						//for(SSPBUF=receive_buffer[++RXptr];!PIR1bits.SSPIF;);		//write
						for(SSPBUF=i;!SSPSTATbits.BF;);		//write
						//WriteSPI(receive_buffer[++RXptr]);		//write
						if(SSPBUF!=i) errors++;
						i++;
					}
					TXins(HIBYTE(errors));
					TXins(LOBYTE(errors));
				}
				else{
					TXins(RX_ERR);
					receive_buffer[RXptr+1]=FLUSH;
				}
				break;
#endif
			default:	//ERR
				TXins(RX_ERR);
				break;
		}
		RXptr++;
	}
	else LED1=0;
}


/******************************************************************************
 * Function:        void TXins(byte)
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:		Write to output queue
 *****************************************************************************/
void TXins(byte x)
{
	if(IN_pending){					//if buffer is full write to a temp buffer
		if(TXaux<TX_TEMP_MAX) transmit_temp[TXaux++]=x;
	}
	else transmit_buffer[TXptr++]=x;
	if(TXptr==HID_INPUT_REPORT_BYTES){
		TXptr=0;
   	   	if(!(ep1Bi.Stat&UOWN)) HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
   	 	else IN_pending=1;
	}
}

/******************************************************************************
 * Function:        void timer_isr(void)
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:		DCDC control function
 * Note:            None
 *****************************************************************************/
#pragma interruptlow timer_isr
void timer_isr (void)
{
#if defined(NO_CCP2)			//no autostart by CCP2
	ADCON0bits.GO=1;			//start conversion
	TMR1H=0xF4;					//64K-3000 @48MHz = 250 us
	TMR1L=0x48;
	while(ADCON0bits.GO);
	PIR1bits.TMR1IF = 0;
#else
	while(ADCON0bits.GO);		//in case ADC is not finished
	PIR1bits.ADIF = 0;
#endif
	errz=err;
	HIBYTE(err)=ADRESH;
	LOBYTE(err)=ADRESL;
#if defined(ADC12)			//must shift if 12 bit result
	err>>=2;
#endif
	err-=vreg;
	//pwm+=errz*225
	//pwm-=err*228
#define F 1
#define W 0
	_asm						//arg2=errz arg1=225
	MOVLW 	225
	MULWF 	errz,0 				// ARG1L * ARG2L -> PRODH:PRODL
	MOVFF 	PRODH, RES1
	MOVFF 	PRODL, RES0
	MOVLW 	225
	MULWF 	errz+1,0			// ARG1L * ARG2H -> PRODH:PRODL
	MOVF 	PRODL, W,0
	ADDWF 	RES1, F,0 			// Add cross
	MOVF 	RES0,W,0			//pwm+=errz*225
	ADDWF	pwm,F,0
	MOVF 	RES1,W,0
	ADDWFC	pwm+1,F,0
	MOVLW 	228
	MULWF 	err,0 				// ARG1L * ARG2L -> PRODH:PRODL
	MOVFF 	PRODH, RES1
	MOVFF 	PRODL, RES0
	MOVLW 	228
	MULWF 	err+1,0				// ARG1L * ARG2H -> PRODH:PRODL
	MOVF 	PRODL, W,0
	ADDWF 	RES1, F,0 			// Add cross
	MOVF 	RES0,W,0			//pwm-=err*228
	SUBWF	pwm,F,0
	MOVF 	RES1,W,0
	SUBWFB	pwm+1,F,0

	_endasm

	if(pwm<=-512) pwm=-512;
	if(pwm>0x6400) pwm=0x6400;
	if(HIBYTE(pwm)<0){
		CCPR1L=0;
		CCP1CON = (CCP1CON & 0xCF);
	}
	else{
		CCPR1L=HIBYTE(pwm);
		CCP1CON = (CCP1CON & 0xCF) | ((LOBYTE(pwm) >> 2) & 0x30);
	}
}
#pragma code

/******************************************************************************
Delay function; waits for N us   (minimum 2 us!)
 *****************************************************************************/
void Delay1us(unsigned char delay){
	byte d=delay-2;
	for(;d;d--){
		Nop();
		Nop();
		Nop();
		Nop();
		Nop();
		Nop();
		Nop();
	}
	return;
}


#if defined(SW_I2C)					//software I2C
// This code is a modified version of MCC18 library code for software I2C
void SWStartI2C( void )
{
  DATA_LAT = 0;                   // set data pin latch to 0
  DATA_LOW;                       // set pin to output to drive low
  DelayT();                   // user may need to modify based on Fosc
}

signed char SWAckI2C( void )
{
  SCLK_LAT = 0;                   // set clock pin latch to 0
  CLOCK_LOW;                      // set clock pin to output to drive low
  DATA_HI;                        // release data line to float high
  DelayT();                   // user may need to modify based on Fosc
  CLOCK_HI;                       // release clock line to float high
  DelayT();                    // 1 cycle delay

  if ( DATA_PIN )                 // error if ack = 1, slave did not ack
  {
    return ( -1 );                // return with acknowledge error
  }
  else
  {
    return ( 0 );                 // return with no error
  }
}

unsigned int SWGetcI2C( void )
{
  BIT_COUNTER = 8;                // set bit count for byte
  SCLK_LAT = 0;                   // set clock pin latch to 0

  do
  {
    CLOCK_LOW;                    // set clock pin output to drive low
    DATA_HI;                      // release data line to float high
    DelayT();                 // user may need to modify based on Fosc
    CLOCK_HI;                     // release clock line to float high
    DelayT();                  // user may need to modify based on Fosc
    I2C_BUFFER <<= 1;             // shift composed byte by 1
    I2C_BUFFER &= 0xFE;           // clear bit 0

    if ( DATA_PIN )               // is data line high
     I2C_BUFFER |= 0x01;          // set bit 0 to logic 1

  } while ( --BIT_COUNTER );      // stay until 8 bits have been acquired

  return ( (unsigned int) I2C_BUFFER ); // return with data
}

signed char SWPutcI2C( unsigned char data_out )
{
  BIT_COUNTER = 8;                // initialize bit counter
  I2C_BUFFER = data_out;          // data to send out
  SCLK_LAT = 0;                   // set latch to 0

  do
    {
     I2C_BUFFER &= 0xFF;          // generate movlb instruction
      _asm
      rlcf I2C_BUFFER,1,1         // rotate into carry and test
      _endasm

      if ( STATUS & 0x01 )        // if carry set, transmit out logic 1
      {
       CLOCK_LOW;                 // set clock pin output to drive low
       DATA_HI;                   // release data line to float high
       DelayT();              // user may need to modify based on Fosc
       CLOCK_HI;                  // release clock line to float high
       DelayT();              // user may need to modify based on Fosc
      }
      else                        // transmit out logic 0
      {
        CLOCK_LOW;                // set clock pin output to drive low
        DATA_LAT = 0;             // set data pin latch to 0
        DATA_LOW;                 // set data pin output to drive low
        DelayT();             // user may need to modify based on Fosc
        CLOCK_HI;                 // release clock line to float high
        DelayT();             // user may need to modify based on Fosc
      }

     BIT_COUNTER --;              // reduce bit counter by 1
  } while ( BIT_COUNTER );        // stay in transmit loop until byte sent

  return ( 0 );                   // return with no error
}

void SWRestartI2C( void )
{
  SCLK_LAT = 0;                   // set clock pin latch to 0
  CLOCK_LOW;                      // set clock pin to output to drive low
  DATA_HI;                        // release data pin to float high
  DelayT();                   // user may need to modify based on Fosc
  CLOCK_HI;                       // release clock pin to float high
  DelayT();                   // user may need to modify based on Fosc
  DATA_LAT = 0;                   // set data pin latch to 0
  DATA_LOW;                       // set data pin output to drive low
  DelayT();                   // user may need to modify based on Fosc
}

void SWStopI2C( void )
{
  SCLK_LAT = 0;                   // set clock pin latch to 0
  CLOCK_LOW;                      // set clock pin to output to drive low
  DATA_LAT = 0;                   // set data pin latch to 0
  DATA_LOW;                       // set data pin output to drive low
  DelayT();                   // user may need to modify based on Fosc
  CLOCK_HI;                       // release clock pin to float high
  DelayT();                   // user may need to modify based on Fosc
  DATA_HI;                        // release data pin to float high
  DelayT();                    // user may need to modify based on Fosc
  DelayT();
}

void NAckI2C( void )
{
      CLOCK_LOW;                  // make clock pin output to drive low
      DATA_HI;                    // release data line to float high
      DelayT();               // user may need to modify based on Fosc
      CLOCK_HI;                   // release clock line to float high
      DelayT();               // user may need to modify based on Fosc
}

void AckI2C( void )
{
      CLOCK_LOW;                  // make clock pin output to drive low
      DATA_LAT = 0;               // set data pin latch to 0
      DATA_LOW;                   // make data pin output to drive low
      DelayT();               // user may need to modify based on Fosc
      CLOCK_HI;                   // release clock line to float high
      DelayT();               // user may need to modify based on Fosc
}


void DelayT(){
	for(tt=T2;tt;tt--){
		Nop();
		Nop();
		Nop();
		Nop();
		Nop();
		Nop();
		Nop();
	}
	return;
}
#endif
#if defined(SW_SPI)					//software SPI

//Transfer one byte over SPI: Clock on RB1, input on RB0, output on RC7
unsigned char SW_IO_SPI(unsigned char c){

	INTCONbits.GIE=0;
	I2C_BUFFER=c;
	RES0=T2;
	_asm
	MOVLW	8
	movwf	i2,0
spi_cycle:

	btfsc 	RES0,2,0			//Check data phase (bit 2 of T2)
	bra 	spi_NOphase0
	BTFSS	I2C_BUFFER,7,0		//MSB first
	BCF 	LATC,7,0	//D0();
	BTFSC 	I2C_BUFFER,7,0
	BSF 	LATC,7,0	//D1();
spi_NOphase0:
	movf	T1,0,0		//half bit delay
	movwf	tt,0
	spi_halfbit0:
	decfsz	tt,1,0
	bra 	spi_halfbit0

	BTFSS 	RES0,3,0		//Clock edge, depending on polarity
	BSF 	LATB,1,0
	BTFSC 	RES0,3,0
	BCF 	LATB,1,0

	btfss 	RES0,2,0			//Check data phase (bit 2 of T2)
	bra 	spi_NOphase1
	BTFSS	I2C_BUFFER,7,0		//MSB first
	BCF 	LATC,7,0	//D0();
	BTFSC 	I2C_BUFFER,7,0
	BSF 	LATC,7,0	//D1();
	bra 	spi_NOphase0in
spi_NOphase1:
	RRCF 	PORTB,0,0	//shift in RB0
	RLCF	I2C_BUFFER,1,0
spi_NOphase0in:

	movf	T1,0,0		//half bit delay
	movwf	tt,0
	spi_halfbit1:
	decfsz	tt,1,0
	bra 	spi_halfbit1

	BTFSS 	RES0,3,0		//Clock edge, depending on polarity
	BCF 	LATB,1,0
	BTFSC 	RES0,3,0
	BSF 	LATB,1,0

	btfss 	RES0,2,0			//Check data phase (bit 2 of T2)
	bra 	spi_NOphase1in
	RRCF 	PORTB,0,0	//shift in RB0
	RLCF	I2C_BUFFER,1,0
spi_NOphase1in:

	decfsz	i2,1,0
	BRA 	spi_cycle
	_endasm
	INTCONbits.GIE=1;
	return I2C_BUFFER;
}
#endif
