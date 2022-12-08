/*
 * File:   i2c_routines.c
 * Author: jhoffmue
 *
 * Created on 24. März 2017, 11:01
 */
#include "i2c_routines.h"


void init_i2c(void)
{
    I2C1BRG = 45;					// I2C Clock setting -> sollte ~400KHz sein bei FCY=40MHz
    I2C1CONbits.I2CEN = 1;
    while(I2C1CONbits.I2CEN == 0);
}

int8_t I2C_start(uint8_t address)
{
    uint16_t timeout = I2C_TIMEOUT;
    
    //Pre clear the intterupt flag
	IFS1bits.MI2C1IF = 0;

	//Send the start condition
	I2C1CONbits.SEN = 1;

	//Wait for the start condition to complete
	while(!IFS1bits.MI2C1IF)
    {
        __delay_us(1);
        if(timeout-- == 0) return -1;
    }
	IFS1bits.MI2C1IF = 0;

	//Send the I2C address
	I2C1TRN = address;

	//Wait for the send to complete and check that the node acknowledged
	timeout = I2C_TIMEOUT;
	while(!IFS1bits.MI2C1IF)
    {
        __delay_us(1);
        if(timeout-- == 0) return -1;
    }
    IFS1bits.MI2C1IF = 0;

	if(I2C1STATbits.ACKSTAT) return 1; //ERROR, node did not acknowledge!!!
    
    return 0;
}

int8_t I2C_write(uint8_t data)
{
    uint16_t timeout = I2C_TIMEOUT;
    
    //Pre clear the intterupt flag
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = data;
    //Wait for the send to complete and check that the node acknowledged
    while(!IFS1bits.MI2C1IF)
    {
        __delay_us(1);
        if(timeout-- == 0) return -1;
    }
    IFS1bits.MI2C1IF = 0;
    if(I2C1STATbits.ACKSTAT) return 1; //ERROR, node did not acknowledge!!!
    
    return 0;
}

int8_t I2C_rep_start(uint8_t address)
{
    uint16_t timeout = I2C_TIMEOUT;

    //Pre clear the intterupt flag
    IFS1bits.MI2C1IF = 0;
    
    I2C1CONbits.RSEN = 1;
    while(!IFS1bits.MI2C1IF)
    {
        __delay_us(1);
        if(timeout-- == 0) return -1;
    }
    IFS1bits.MI2C1IF = 0;
    
    //Send the I2C address
	I2C1TRN = address;

	//Wait for the send to complete and check that the node acknowledged
	timeout = I2C_TIMEOUT;
    while(!IFS1bits.MI2C1IF)
	{
        __delay_us(1);
        if(timeout-- == 0) return -1;
    }
    IFS1bits.MI2C1IF = 0;

	if(I2C1STATbits.ACKSTAT) return 1; //ERROR, node did not acknowledge!!!
    
    return 0;
}

int8_t I2C_read(uint8_t number , uint8_t *values)
{
    uint16_t timeout = I2C_TIMEOUT;
    uint8_t i = 0;
    
    for(i=0;i<number;i++)
    {
        //Pre clear the interrupt flag
        IFS1bits.MI2C1IF = 0;
        
        I2C1CONbits.RCEN = 1;
        while(!IFS1bits.MI2C1IF)
        {
            __delay_us(1);
            if(timeout-- == 0) return -1;
        }
        IFS1bits.MI2C1IF = 0;
        
        if(i == (number-1)) I2C1CONbits.ACKDT = 1;
        else                I2C1CONbits.ACKDT = 0;
        I2C1CONbits.ACKEN = 1;
        timeout = I2C_TIMEOUT;
        while(!IFS1bits.MI2C1IF)
        {
        __delay_us(1);
        if(timeout-- == 0) return -1;
        }
        IFS1bits.MI2C1IF = 0;
    
        values[i] = I2C1RCV;
    }
    return 0;
}

int8_t I2C_stop(void)
{
    uint16_t timeout = I2C_TIMEOUT;

    //Pre clear the intterupt flag
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.PEN = 1;
    while(!IFS1bits.MI2C1IF)
    {
        __delay_us(1);
        if(timeout-- == 0) return -1;
    }
    IFS1bits.MI2C1IF = 0;
    
    return 0;
}

