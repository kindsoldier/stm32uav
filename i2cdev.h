/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#ifndef I2CDEV_H_QWERTY
#define I2CDEV_H_QWERTY

#include <stdint.h>

void i2cdev_write_reg8(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t value);
uint8_t i2cdev_read_reg8(uint32_t i2c, uint8_t addr, uint8_t reg);

void i2cdev_read_seq8(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t size);

void i2cdev_reg_setbits(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t mask);
void i2cdev_reg_cleanbits(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t mask);

#endif
