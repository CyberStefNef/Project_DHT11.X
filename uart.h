#ifndef _UART_H_
   #define _UART_H_

extern void CommInit(void);
extern void CommPutRxChar(unsigned char data);
extern unsigned char CommGetChar(void);
extern unsigned char CommIsEmpty(void);
extern unsigned char CommGetTxChar(void);
extern void CommPutChar(unsigned char data);
extern void CommPutString(unsigned char *str_data);

extern void __attribute__ ((interrupt, no_auto_psv)) _U1TXInterrupt(void);
extern void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void);
extern unsigned int PulsWidth;
#endif // _init_device_H_
