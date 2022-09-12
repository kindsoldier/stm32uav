/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdint.h>
#include <geometry.h>

typedef struct {
    uint32_t bus;
    uint8_t  addr;
    double   gxe;
    double   gye;
    double   gze;
} imu_t;

void imu_setup(imu_t* imu, uint32_t i2c, uint8_t addr);
void imu_calibrate(imu_t* imu, int count);
void imu_gettilt(imu_t* imu, int loops, eulerangle_t* a);
void imu_getvec(imu_t* imu, imuvec_t* val);
