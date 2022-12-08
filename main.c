/*
 * File:   main.c
 * Author: stefm
 *
 * Created on November 29, 2022, 8:59 AM
 */
#define FCY 40000000ULL 

#include "config.h"
#include <p33FJ128GP802.h>
#include <libpic30.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "i2c_routines.h"
#include "OLED.h"
#include <stdio.h>
#include "uart.h"
#include "uart_dsPIC.h"  
#include<string.h>

// DHT11 struct
struct DHT11
{
    int humidity;
    int humidity_decimal;
    int temperature;
    int temperature_decimal;
    int checksum;
};

struct threshold
{
    int border1;
    int border2;
} Threshold;
struct DHT11 SensorData;

void init_system_clock()
{
	CLKDIVbits.ROI = 0;
	CLKDIVbits.DOZE = 0;
	CLKDIVbits.DOZEN = 0;
	CLKDIVbits.FRCDIV = 0;
	CLKDIVbits.PLLPOST = 0;	// 0 - 31 N1=<value>+2
	CLKDIVbits.PLLPRE = 0;	// 0: N2=2 ; 1: N2=4 ; 3: N2=8
	
	PLLFBDbits.PLLDIV = 78; // 0-511 M=<value>+2			// 80MHz -> FCY = 40MHz
	
	while(!OSCCONbits.LOCK);    // wait for PLL ready
}

struct DHT11 getData(void);
unsigned int time_us(unsigned int time);
unsigned int time_ms(unsigned int time);
unsigned int t1, t2;

char checkResponse = 0;

int main(int argc, char **argv)
{
    
    init_system_clock();
    
    AD1PCFGL = 0xFFFF;
    TRISB = 0;
    
    init_i2c();
    init_OLED();
    
    CommConfig();               // Configure UART1
    CommInit();
    CommEnable();               // Enable UART1
    
    CommPutString("Ready to set Threshold enter 'value1#value2'\r");

    initTimer2();
    initInputCapture();
    
    // Etwas Text in den Framebuffer schreiben
    fb_draw_string_big(35,0,"FH KIEL");
    fb_show();
    
    Threshold.border1 = 50;
    Threshold.border2 = 70;
    
    unsigned char data = 0;

    while (1)
    {
        
        int string_index = 0;
        char char_array[8] = {0};
        while (CommIsEmpty() != 1){  // Echo of RX
            data = CommGetChar();
            char_array[string_index] = data;
            string_index++;
        }
        
        char *ptr = strtok(char_array, "#");
        string_index = 0;
        while(ptr != NULL)
        {
            if(string_index == 0) {
                int value;
                value = atoi(ptr);
                Threshold.border1 = value;
                char str_buf[50];
                sprintf(str_buf, "Received lower Threshold: %d \r", value);
                CommPutString(str_buf);
            } else if(string_index == 1) {
                int value;
                value = atoi(ptr);
                Threshold.border2 = value;
                char str_buf[50];
                sprintf(str_buf, "Received upper Threshold: %d \r", value);
                CommPutString(str_buf);
            }
            ptr = strtok(NULL, "#");
            string_index++;
        }

        
        sendTrigger();
        if (checkResponse == 1)
        {         
            stopIC1();
            char temp_string[100];
            sprintf(temp_string, "TEM: %dC", SensorData.temperature);
            fb_draw_string_big(25,2,temp_string);
            
            char humidity_string[100];
            sprintf(humidity_string, "HUM: %d%", SensorData.humidity);
            fb_draw_string_big(25,4,humidity_string);
            fb_show();
            
            if(SensorData.humidity <= Threshold.border1) {
                LATBbits.LATB12 = 0;
                LATBbits.LATB13 = 0;
                LATBbits.LATB14 = 1;
            } else if(SensorData.humidity > Threshold.border1 && SensorData.humidity <= Threshold.border2) {
                LATBbits.LATB12 = 0;
                LATBbits.LATB13 = 1;
                LATBbits.LATB14 = 0;
            } else if(SensorData.humidity > Threshold.border2){
                LATBbits.LATB12 = 1;
                LATBbits.LATB13 = 0;
                LATBbits.LATB14 = 0;
            }
            
            checkResponse = 0;
        } else {
            t1 = 0;
            t2 = 0;
        }
        __delay_ms(1000);
    }
}

// send data to trigger dht11 on port B pin 15
void sendTrigger()
{
    stopIC1();
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;
    __delay_ms(20);
    LATBbits.LATB15 = 1;
    __delay_us(30);
    TRISBbits.TRISB15 = 1;
    startIC1();
}

// init Port B pin RB15 for input change notification
void initInputCapture(void)
{
    IC1CONbits.ICM = 0b000;
    __builtin_write_OSCCONL(OSCCONL & 0xBF);
    RPINR7bits.IC1R = 0b1111;
    __builtin_write_OSCCONL(OSCCONL | 0x40);
    IC1CONbits.ICSIDL = 0;
    IC1CONbits.ICTMR = 1;
    IC1CONbits.ICI = 0b00;
    IC1CONbits.ICBNE = 0;

    IPC0bits.IC1IP = 0b011;
    IFS0bits.IC1IF = 0;
    IEC0bits.IC1IE = 1;
}

void startIC1(void)
{
    IC1CONbits.ICM = 0b010;
    TMR2 = 0x00;
}

void stopIC1(void)
{
    IC1CONbits.ICM = 0b000;
}

void initTimer2(void)
{
    T2CONbits.TON = 0;
    T2CONbits.TGATE = 0;
    T2CONbits.TCKPS = 0b01;
    T2CONbits.TCS = 0;

    TMR2 = 0x00;

    T2CONbits.TON = 1;
}

unsigned int timePeriod = 0;
int index = 0;
int SensorRaw[40];
// interrupt handler input change notification
void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{
    // clear interrupt flag
    IFS0bits.IC1IF = 0;

    t2 = IC1BUF;
    if (t1 == 0)
        timePeriod = time_us(t2);
    else if (t2 > t1)
        timePeriod = time_us(t2 - t1);
    else
        timePeriod = time_us((PR2 - t1) + t2);

    if (checkResponse)
    {
        if (timePeriod > 66 && timePeriod < 88)
            SensorRaw[index] = 0;
        else if (timePeriod > 110 && timePeriod < 130)
            SensorRaw[index] = 1;
        index++;
    }

    if (timePeriod > 150 && timePeriod < 170)
        checkResponse = 1;

    t1 = t2;
    if(index == 40) {
        SensorData = getData();
        
        while(IC1CONbits.ICBNE != 0){
            t2 = IC1BUF;
        }
        index = 0;
        t1 = 0;
        t2 = 0;
    }
}

unsigned int time_us(unsigned int time)
{
    return time * (unsigned int)8 / 40;
}

unsigned int time_ms(unsigned int time)
{
    return time_us(time) / 1000;
}

// Function that converts an 40bit array to a struct
struct DHT11 getData(void)
{
    struct DHT11 dht11 = {0,0,0,0,0};
    int i = 0;
    for (i = 0; i < 8; i++)
    {
        dht11.humidity += SensorRaw[i] * pow(2, 7 - i);
        SensorRaw[i] = 0;
    }
    for (i = 8; i < 16; i++)
    {
        dht11.humidity_decimal += SensorRaw[i] * pow(2, 15 - i);
        SensorRaw[i] = 0;
    }
    for (i = 16; i < 24; i++)
    {
        dht11.temperature += SensorRaw[i] * pow(2, 23 - i);
        SensorRaw[i] = 0;
    }
    for (i = 24; i < 32; i++)
    {
        dht11.temperature_decimal += SensorRaw[i] * pow(2, 31 - i);
        SensorRaw[i] = 0;
    }
    for (i = 32; i < 40; i++)
    {
        dht11.checksum += SensorRaw[i] * pow(2, 39 - i);
        SensorRaw[i] = 0;
    }
    int check = (dht11.humidity + dht11.humidity_decimal + dht11.temperature+ dht11.temperature_decimal);
    if(dht11.checksum != check) {
        checkResponse = 0;
         struct DHT11 dht11error = {0,0,0,0,0};
        return dht11error;
    }
    return dht11;
}