/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <libopencm3/stm32/i2c.h>
#include <stdint.h>

#include <mpu6050.h>

#define MPU_REG_CONFIG           0x1A
#define MPU_REG_GYRO_CONFIG      0x1B
#define MPU_REG_ACCEL_CONFIG     0x1C

#define MPU_REG_ACCEL_XOUT_H     0x3B
#define MPU_REG_ACCEL_XOUT_L     0x3C
#define MPU_REG_ACCEL_YOUT_H     0x3D
#define MPU_REG_ACCEL_YOUT_L     0x3E
#define MPU_REG_ACCEL_ZOUT_H     0x3F
#define MPU_REG_ACCEL_ZOUT_L     0x40
#define MPU_REG_TEMP_OUT_H       0x41
#define MPU_REG_TEMP_OUT_L       0x42
#define MPU_REG_GYRO_XOUT_H      0x43
#define MPU_REG_GYRO_XOUT_L      0x44
#define MPU_REG_GYRO_YOUT_H      0x45
#define MPU_REG_GYRO_YOUT_L      0x46
#define MPU_REG_GYRO_ZOUT_H      0x47
#define MPU_REG_GYRO_ZOUT_L      0x48

#define MPU_REG_PWR_MGMT_1       0x6B
#define MPU_REG_PWR_MGMT_2       0x6C

/* GYRO_CONFIG 0x1B */
#define MPU_GYRO_FS_BASE        3
#define MPU_GYRO_FS_LEN         2

#define MPU_GYRO_FS_250         0
#define MPU_GYRO_FS_500         1
#define MPU_GYRO_FS_1000        2
#define MPU_GYRO_FS_2000        3

/* ACCEL_CONFIG 0x1C */
#define MPU_ACCEL_FS_BASE       3
#define MPU_ACCEL_FS_LEN        2
#define MPU_ACCEL_FS_2          0
#define MPU_ACCEL_FS_4          1
#define MPU_ACCEL_FS_8          2
#define MPU_ACCEL_FS_16         3

#define MPU_GYRO_LSB_250    131.0f
#define MPU_GYRO_LSB_500     65.5f
#define MPU_GYRO_LSB_1000    32.8f
#define MPU_GYRO_LSB_2000    16.4f

#define MPU_ACCEL_LSB_2     16384.0f
#define MPU_ACCEL_LSB_4      8192.0f
#define MPU_ACCEL_LSB_8      4096.0f
#define MPU_ACCEL_LSB_16     2048.0f


//static uint8_t i2cdev_read_reg8(uint32_t i2c, uint8_t addr, uint8_t reg);
static void i2cdev_write_reg8(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t value);
static void i2cdev_read_seq8(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t size);

static void i2cdev_write_reg8(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    i2c_transfer7(i2c, addr, buffer, 2, NULL, 0);
}

//static void i2cdev_reg_setbit(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t mask) {
//    uint8_t buffer[2];
//    buffer[0] = reg;
//    buffer[1] = 0x00;
//    i2c_transfer7(i2c, addr, &buffer[0], 1, &buffer[1], 1);
//    buffer[1] |= mask;
//    i2c_transfer7(i2c, addr, buffer, 2, NULL, 0);
//}

//static uint8_t i2cdev_read_reg8(uint32_t i2c, uint8_t addr, uint8_t reg) {
//    uint8_t val;
//    i2c_transfer7(i2c, addr, &reg, 1, &val, 1);
//    return val;
//}


static void i2cdev_read_seq8(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t size) {
    i2c_transfer7(i2c, addr, &reg, 1, buffer, size);
}


void mpu_setup(mpu_t* mpu, uint32_t i2c, uint8_t addr) {

    mpu->bus    = i2c;
    mpu->addr   = addr;

    mpu->err.ax = 0;
    mpu->err.ay = 0;
    mpu->err.az = 0;
    mpu->err.gx = 0;
    mpu->err.gy = 0;
    mpu->err.gz = 0;

    i2cdev_write_reg8(i2c, addr, MPU_REG_PWR_MGMT_1, 0x00);
    i2cdev_write_reg8(i2c, addr, MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_250 << MPU_GYRO_FS_BASE);
    i2cdev_write_reg8(i2c, addr, MPU_REG_ACCEL_CONFIG, MPU_ACCEL_FS_2 << MPU_ACCEL_FS_BASE);
}

static void mpu_rawread(mpu_t* mpu, mpu_value_t* val) {

    uint8_t buffer[14];
    i2cdev_read_seq8(mpu->bus, mpu->addr, MPU_REG_ACCEL_XOUT_H, (uint8_t*)buffer, 14);

    int16_t ax = (((int16_t) buffer[0]) << 8) | buffer[1];
    int16_t ay = (((int16_t) buffer[2]) << 8) | buffer[3];
    int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];

    int16_t gx = (((int16_t) buffer[8]) << 8) | buffer[9];
    int16_t gy = (((int16_t) buffer[10]) << 8) | buffer[11];
    int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

    val->ax = (float)ax / MPU_ACCEL_LSB_2;
    val->ay = (float)ay / MPU_ACCEL_LSB_2;
    val->az = (float)az / MPU_ACCEL_LSB_2;

    val->gx = (float)gx / MPU_GYRO_LSB_250;
    val->gy = (float)gy / MPU_GYRO_LSB_250;
    val->gz = (float)gz / MPU_GYRO_LSB_250;

    //val->gx *= 0.0174533f;
    //val->gy *= 0.0174533f;
    //val->gz *= 0.0174533f;

}

void mpu_calibrate(mpu_t* mpu, int count) {
    mpu_value_t val;
    val.ax = 0;
    val.ay = 0;
    val.az = 0;
    val.gx = 0;
    val.gy = 0;
    val.gz = 0;

    for (int i = 0; i < count; i++) {
        mpu_rawread(mpu, &val);

        mpu->err.ax += val.ax;
        mpu->err.ay += val.ay;
        mpu->err.az += val.az;

        mpu->err.gx += val.gx;
        mpu->err.gy += val.gy;
        mpu->err.gz += val.gz;
    }

    mpu->err.ax /= count;
    mpu->err.ay /= count;
    mpu->err.az /= count;

    mpu->err.gx /= count;
    mpu->err.gy /= count;
    mpu->err.gz /= count;
}


void mpu_read(mpu_t* mpu, mpu_value_t* val) {
    mpu_rawread(mpu, val);

    val->ax -= mpu->err.ax;
    val->ay -= mpu->err.ay;
    val->az -= mpu->err.az;

    val->gx -= mpu->err.gx;
    val->gy -= mpu->err.gy;
    val->gz -= mpu->err.gz;

}
