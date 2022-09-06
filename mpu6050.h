/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdint.h>

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} mpu_value_t;

typedef struct {
    uint32_t    bus;
    uint8_t     addr;
    mpu_value_t err;
} mpu_t;

void mpu_setup(mpu_t* mpu, uint32_t i2c, uint8_t addr);
void mpu_calibrate(mpu_t* mpu, int count);
void mpu_read(mpu_t* mpu, mpu_value_t* val);
