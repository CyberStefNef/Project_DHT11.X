
#include "xc.h"	    // Einbinden der prozessor-spezifischen 
                                // Header-Datei
#include "uart.h"               // Access to the Ring Buffer

// Defines
//#define BAUDRATEREG1 12         // 9600 Baud, BRGH=0, fOSC = 4MHz 
                                // ohne PLL
#define BAUDRATEREG1 21         // 115200 Baud, BRGH=0, fOSC = 80MHz 
                                // mit PLL

#define U1RXPIN RPINR18bits.U1RXR = 3;// RP3 (Pin 7) to U1RX
#define U1RXTRIS TRISBbits.TRISB3 = 1;// RB3 (RP3) as Input für U1RX
#define U1TXPIN RPOR1bits.RP2R = 3;   // U1TX to RP2 (Pin 6)

/****** Interrupt Service Routines ******/

void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void) 
{
    unsigned char data;
	
	data = U1RXREG;         // Read byte from UART RX FIFO
	CommPutRxChar(data);    // Write received byte to RX Ring Buffer
	IFS0bits.U1RXIF = 0;    // Clear IRQ Flag
}
void __attribute__ ((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
	unsigned char data;

	data = CommGetTxChar();     // Read byte from TX Ring Buffer
	U1TXREG = data;             // Write to TX FIFO
	
	IFS0bits.U1TXIF = 0;        // Clear IRQ Flag
}

/****** UART Functions ******/


void CommConfig(void) 
{
   
	// The UART is configured to be duplex wit 8 data byte, no parity and
	// one stop bit. There is no flow control and only Rx & Tx.
	// The baudrate is set via a define BAUDRATEREG1

	// configure U1MODE
	U1MODEbits.UARTEN   = 0;    // Bit15 TX, RX DISABLED, ENABLE at end of func
	U1MODEbits.USIDL    = 0;    // Bit13 Continue in Idle
	U1MODEbits.IREN     = 0;    // Bit12 No IRDA translation
	U1MODEbits.RTSMD    = 0;    // Bit11 RTS Mode (not used)
	U1MODEbits.UEN      = 0;	// Bits8,9 TX,RX enabled, CTS,RTS, BCLK not
	U1MODEbits.WAKE     = 0;	// Bit7 No Wake up (since we don't sleep here)
	U1MODEbits.LPBACK   = 0;	// Bit6 No Loop Back
	U1MODEbits.ABAUD    = 0;	// Bit5 No Autobaud (would require sending '55')
	U1MODEbits.URXINV    = 0;	// Bit4 IdleState = 1
	U1MODEbits.BRGH     = 0;	// Bit3 16 clocks per bit period
	U1MODEbits.PDSEL    = 0;	// Bits1,2 8bit, No Parity
	U1MODEbits.STSEL    = 0;	// Bit0 One Stop Bit
	
	U1BRG = BAUDRATEREG1;	    // baud rate

	// Load all values in for U1STA SFR
	U1STAbits.UTXISEL1  = 0;	//Bit15 TX IRQ when Char is transferred 
	U1STAbits.UTXINV    = 0;	//Bit14 No polarity inversion
	U1STAbits.UTXISEL0  = 0;	//Bit13 Other half of Bit15
	U1STAbits.UTXBRK    = 0;	//Bit11 TX Break Disabled
	U1STAbits.UTXEN     = 0;	//Bit10 TX disabled, pins controlled by Port
	U1STAbits.UTXBF     = 0;	//Bit9 *Read Only Bit* TX FIFO Full
	U1STAbits.TRMT      = 0;	//Bit8 *Read Only bit* TSR empty
	U1STAbits.URXISEL   = 0;	//Bits6,7 RX IRQ on character received
	U1STAbits.ADDEN     = 0;	//Bit5 Address Detect Disabled
	U1STAbits.RIDLE     = 0;	//Bit4 *Read Only Bit* RX idle
	U1STAbits.PERR      = 0;	//Bit3 *Read Only Bit* Parity error
	U1STAbits.FERR      = 0;	//Bit2 *Read Only Bit* Framing error
	U1STAbits.OERR      = 0;	//Bit1 *Read Only Bit* Overflow error
	U1STAbits.URXDA     = 0;	//Bit0 *Read Only Bit* RX FIFO has data

	IFS0bits.U1TXIF     = 0;	// Clear the Transmit Interrupt Flag
//	IEC0bits.U1TXIE     = 1;	// Enable Transmit Interrupts //
	IFS0bits.U1RXIF     = 0;	// Clear the Receive Interrupt Flag
	IEC0bits.U1RXIE     = 1;	// Enable Receive Interrupts
	
	
	// Select the IO Pins for the UART
	__builtin_write_OSCCONL(OSCCONL & 0xBF); // Unlock IOLOCK

    U1RXPIN             // Select RX Pin
    U1TXPIN             // Select TX Pin

    __builtin_write_OSCCONL(OSCCONL | 0x40); // Lock IOLOCK

    U1RXTRIS            // Set RX pin as input

}

void CommEnable(void)
{
	U1MODEbits.UARTEN   = 1;	// And turn the peripheral on
	U1STAbits.UTXEN     = 1;    // Enable TX after turning UART on
}

void CommDisable(void)
{
    U1MODEbits.UARTEN   = 0;	// And turn the peripheral on
    U1STAbits.UTXEN     = 0;    // Disable TX after turning UART off
}

unsigned char CommTxFull(void)
// Checks if the UART Tx FIFO is full
{
    unsigned char flag;
    
    if (U1STAbits.UTXBF == 1)
        flag = 1;
    else
        flag = 0;
    
    return (flag);
}            

void CommRxFlush(void)
// empties the RX FIFO
{
    unsigned char data;
    
    while(U1STAbits.URXDA == 1) // Loop while RX FIFO not empty
        data = U1RXREG;         // read FIFO and discard
}    

void CommRxIntEn(void)
// Enables the RX IRQ
{
	IEC0bits.U1RXIE     = 1;	// Enable Receive Interrupts   
}

void CommRxIntDis(void)
// Disables the RX IRQ
{
	IEC0bits.U1RXIE     = 0;	// Disable Receive Interrupts   
}

void CommTxIntEn(void)
// Enables the TX IRQ
{
	IEC0bits.U1TXIE     = 1;	// Enable Transmit Interrupts
}

void CommTxIntDis(void)
// Disables the TX IRQ
{
	IEC0bits.U1TXIE     = 0;	// Disable Transmit Interrupts
}

void CommTxIntSet(void)
// Sets the TX IRQ Flag
{
    IFS0bits.U1TXIF = 1;
}    

/*unsigned char CommTxIntDis2(void)    
{
    unsigned char altIRQ;
    
    altIRQ = IEC0bits.U1TXIE;
    IEC0bits.U1TXIE     = 0;	// Disable Transmit Interrupts
    
    return(altIRQ);
}*/
