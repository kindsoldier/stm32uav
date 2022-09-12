/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include <libopencm3/stm32/i2c.h>
#include <stdint.h>
#include <math.h>

#include <i2cdev.h>
#include <mpu6050.h>

#define MPU_REG_SMPLRT_DIV       0x19

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

/* PWR_MGMT_1 0x6B */
#define MPU_PWR1_DEVICE_RESET_BIT   7
#define MPU_PWR1_SLEEP_BIT          6
#define MPU_PWR1_CYCLE_BIT          5
#define MPU_PWR1_TEMP_DIS_BIT       3

#define MPU_PWR1_CLKSEL_BASE        0
#define MPU_PWR1_CLKSEL_LEN         3

#define MPU_PWR1_CLKSEL_INTERNAL          0
#define MPU_PWR1_CLKSEL_PLL_XGYRO         1
#define MPU_PWR1_CLKSEL_PLL_YGYRO         2
#define MPU_PWR1_CLKSEL_PLL_ZGYRO         3
#define MPU_PWR1_CLKSEL_PLL_EXT32K        4
#define MPU_PWR1_CLKSEL_PLL_EXT19M        5
#define MPU_PWR1_CLKSEL_KEEP_RESET        7

/* PWR_MGMT_2 0x6C */
#define MPU_PWR2_LP_WAKE_CTRL_BASE      6
#define MPU_PWR2_LP_WAKE_CTRL_LEN       2

#define MPU_PWR2_WAKE_FREQ_1P25         0
#define MPU_PWR2_WAKE_FREQ_2P5          1
#define MPU_PWR2_WAKE_FREQ_5            2
#define MPU_PWR2_WAKE_FREQ_10           3

#define MPU_PWR2_STBY_XA_BIT            5
#define MPU_PWR2_STBY_YA_BIT            4
#define MPU_PWR2_STBY_ZA_BIT            3
#define MPU_PWR2_STBY_XG_BIT            2
#define MPU_PWR2_STBY_YG_BIT            1
#define MPU_PWR2_STBY_ZG_BIT            0

#define MPU_GYRO_LSB   MPU_GYRO_LSB_1000
#define MPU_GYRO_FS    MPU_GYRO_FS_1000

#define MPU_ACCEL_LSB  MPU_ACCEL_LSB_16
#define MPU_ACCEL_FS   MPU_ACCEL_FS_16

void imu_setup(imu_t* imu, uint32_t i2c, uint8_t addr) {

    imu->bus    = i2c;
    imu->addr   = addr;

    imu->gxe = 0;
    imu->gye = 0;
    imu->gze = 0;

    //i2cdev_write_reg8(i2c, addr, MPU_REG_PWR_MGMT_1, 1 << MPU_PWR1_DEVICE_RESET_BIT);
    //for (int i = 0; i < 10000; i++) __asm__("nop");

    i2cdev_write_reg8(i2c, addr, MPU_REG_PWR_MGMT_1, 0x00);

    i2cdev_write_reg8(i2c, addr, MPU_REG_GYRO_CONFIG, MPU_GYRO_FS << MPU_GYRO_FS_BASE);
    i2cdev_write_reg8(i2c, addr, MPU_REG_ACCEL_CONFIG, MPU_ACCEL_FS << MPU_ACCEL_FS_BASE);
    i2cdev_write_reg8(i2c, addr, MPU_REG_SMPLRT_DIV, 4);
}


static void imu_rawread(imu_t* imu, imuvec_t* val) {

    uint8_t buffer[14];
    i2cdev_read_seq8(imu->bus, imu->addr, MPU_REG_ACCEL_XOUT_H, (uint8_t*)buffer, 14);

    int16_t ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    int16_t ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    int16_t az = (((int16_t)buffer[4]) << 8) | buffer[5];

    int16_t gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    int16_t gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];

    val->ax = (double)ax / (double)MPU_ACCEL_LSB;
    val->ay = (double)ay / (double)MPU_ACCEL_LSB;
    val->az = (double)az / (double)MPU_ACCEL_LSB;

    val->gx = (double)gx / (double)MPU_GYRO_LSB;
    val->gy = (double)gy / (double)MPU_GYRO_LSB;
    val->gz = (double)gz / (double)MPU_GYRO_LSB;

    val->gx *= M_PI / 180.0;
    val->gy *= M_PI / 180.0;
    val->gz *= M_PI / 180.0;

}

void imu_calibrate(imu_t* imu, int loops) {
    imuvec_t val;
    val.ax = 0;
    val.ay = 0;
    val.az = 0;
    val.gx = 0;
    val.gy = 0;
    val.gz = 0;

    for (int i = 0; i < loops; i++) {
        imu_rawread(imu, &val);
        imu->gxe += val.gx / (double)loops;
        imu->gye += val.gy / (double)loops;
        imu->gze += val.gz / (double)loops;
    }
}

void imu_gettilt(imu_t* imu, int loops, eulerangle_t* a) {
    imuvec_t val;
    val.ax = 0;
    val.ay = 0;
    val.az = 0;
    val.gx = 0;
    val.gy = 0;
    val.gz = 0;

    double ax = 0;
    double ay = 0;
    double az = 0;

    for (int i = 0; i < loops; i++) {
        imu_rawread(imu, &val);
        ax += val.ax / (double)loops;
        ay += val.ay / (double)loops;
        az += val.az / (double)loops;
    }
    a->x = atan(ax / sqrt(ay*ay + az*az));
    a->y = atan(ay / sqrt(ax*ax + az*az));
    a->z = atan(az / sqrt(ax*ax + ay*ay));
}

void imu_getvec(imu_t* imu, imuvec_t* val) {
    imu_rawread(imu, val);
    val->gx -= imu->gxe;
    val->gy -= imu->gye;
    val->gz -= imu->gze;
}
