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

/** P U B L I C  P R O T O T Y P E S *****************************************/
void UserInit(void);
void ProcessIO(void);
void interrupt low_priority timer_isr (void);