#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_write_register(uint8_t devAddr, uint8_t reg, uint8_t data);
uint8_t i2c_read_register(uint8_t devAddr, uint8_t reg);
void i2c_read_bytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, uint8_t length);

#endif 