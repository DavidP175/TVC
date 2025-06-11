#include "I2C.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void i2c_write_register(uint8_t devAddr, uint8_t reg, uint8_t data){
    HAL_I2C_Mem_Write(&hi2c1, (devAddr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

uint8_t i2c_read_register(uint8_t devAddr, uint8_t reg){
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, (devAddr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}


void i2c_read_bytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, uint8_t length){
    HAL_I2C_Mem_Read(&hi2c1, (devAddr<<1), reg, I2C_MEMADD_SIZE_8BIT, buffer, length, 100);
}


