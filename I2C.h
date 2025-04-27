#ifndef I2C_H
#define I2C_H

#include <avr/io.h>

void i2cInit(void);
void i2cStart(void);
void i2cStop(void);
void i2cWrite(uint8_t data);
uint8_t i2cRead_ack(void);
uint8_t i2cRead_nack(void);
void i2c_write_register(uint8_t devAddr, uint8_t reg, uint8_t data);
uint8_t i2c_read_register(uint8_t devAddr, uint8_t reg);
void i2c_read_bytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, uint8_t length);

#endif