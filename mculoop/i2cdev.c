/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <libopencm3/stm32/i2c.h>
#include <stdint.h>
#include <math.h>


void i2cdev_write_reg8(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    i2c_transfer7(i2c, addr, buffer, 2, NULL, 0);
}

uint8_t i2cdev_read_reg8(uint32_t i2c, uint8_t addr, uint8_t reg) {
    uint8_t val;
    i2c_transfer7(i2c, addr, &reg, 1, &val, 1);
    return val;
}

void i2cdev_reg_setbits(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t mask) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = 0x00;
    i2c_transfer7(i2c, addr, &buffer[0], 1, &buffer[1], 1);
    buffer[1] |= mask;
    i2c_transfer7(i2c, addr, buffer, 2, NULL, 0);
}

void i2cdev_reg_cleanbits(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t mask) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = 0x00;
    i2c_transfer7(i2c, addr, &buffer[0], 1, &buffer[1], 1);
    buffer[1] &= ~mask;
    i2c_transfer7(i2c, addr, buffer, 2, NULL, 0);
}

void i2cdev_read_seq8(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t size) {
    i2c_transfer7(i2c, addr, &reg, 1, buffer, size);
}
