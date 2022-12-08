#define FCY 40000000ULL
#include "PIC_init.h"

// Configuration Bits setzen -> aus MPLABX - Production - Set Configuration Bits
// DSPIC33FJ128GP802 Configuration Bit Settings

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC           // Oscillator Mode (Primary Oscillator (XT, HS, EC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


void init_system_clock()
{
	// ACHTUNG: neues MIK-Board läuft mit 4MHz, altes (kleines) Board mit 8MHz Quarz F_PLL_in
	// F_PLL_out: F_PLL_in * ( M / (N1*N2) )	ACHTUNG: FCY = F_PLL_out / 2
	CLKDIVbits.ROI = 0;
	CLKDIVbits.DOZE = 0;
	CLKDIVbits.DOZEN = 0;
	CLKDIVbits.FRCDIV = 0;
	CLKDIVbits.PLLPOST = 0;	// 0 - 31 N1=<value>+2
	CLKDIVbits.PLLPRE = 0;	// 0: N2=2 ; 1: N2=4 ; 3: N2=8
	
	PLLFBDbits.PLLDIV = 78; // 0-511 M=<value>+2			// 80MHz -> FCY = 40MHz
    
    __builtin_write_OSCCONH(0x3);
    __builtin_write_OSCCONL(0x1);
	
    while(OSCCONbits.COSC != 0b011);
	while(!OSCCONbits.LOCK);    // wait for PLL ready
}

void init_ports()
{
	AD1PCFGL = 0xFFFF;		// Alle Eingänge digital !!!Dieses Register muss immer gesetzt werden, unterschiedliche initialisierung in DEBUG und RELEASE
	
	TRISB = 0;				// complete PORTB OUT
	
}

