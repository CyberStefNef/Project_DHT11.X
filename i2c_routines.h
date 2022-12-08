#ifndef I2C_ROUTINES_H
#define	I2C_ROUTINES_H

#define FCY 40000000ULL

#define I2C_TIMEOUT 10000       // timeout in µs


#include <xc.h>
#include <stdint.h>
#include <libpic30.h>
#include <stdint.h>

// function prototypes
void init_i2c(void);
int8_t I2C_start(uint8_t address);
int8_t I2C_write(uint8_t data);
int8_t I2C_rep_start(uint8_t address);
int8_t I2C_read(uint8_t number , uint8_t *values);
int8_t I2C_stop(void);

    
#endif	// I2C_ROUTINES_H
