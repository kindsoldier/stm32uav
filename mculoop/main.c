
/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "usartu.h"
#include "mpu6050.h"

uint32_t g_sys_tick_counter;

static void _delay(uint32_t n) {
    for (int i = 0; i < n * 925; i++)
        __asm__("nop");
}

static void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_I2C1);

}

static void usart_setup(void) {
    usart_disable(USART1);
    nvic_disable_irq(NVIC_USART1_IRQ);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO9 | GPIO10);

    //usart_set_baudrate(USART1, 115200);
    //usart_set_baudrate(USART1, 230400);
    usart_set_baudrate(USART1, 460800);

    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    usart_disable_rx_interrupt(USART1);

    usart_enable(USART1);
}

const uint32_t systic_freq = 50 * 1000;

static void systick_setup(void) {
    g_sys_tick_counter = 0;

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6);

    systick_set_frequency(systic_freq, rcc_ahb_frequency);
    systick_interrupt_enable();
    systick_counter_enable();
}


static void i2c_setup(void) {
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8 | GPIO9);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO8 | GPIO9);
    gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);

    i2c_reset(I2C1);
    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, I2C_CR2_FREQ_36MHZ);

    i2c_peripheral_enable(I2C1);
}

void sys_tick_handler(void) {
    g_sys_tick_counter++;
}

uint32_t sys_tick_counter(void) {
    uint32_t val = g_sys_tick_counter;
    return val;
}

typedef struct angle_s {
    double z;   // yaw
    double y;   // pitch
    double x;   // roll
} angle_t;


typedef struct quaternion_s {
    double w;
    double x;
    double y;
    double z;
} quaternion_t;


double invSqrt(double x) {
    return 1.0f / sqrt(x);
}

void ahrs_update(double dt, quaternion_t* q, mpu_value_t* m) {

    double q0 = q->w;
    double q1 = q->x;
    double q2 = q->y;
    double q3 = q->z;      // quaternion of sensor frame relative to auxiliary frame

    double gx = m->gx;
    double gy = m->gy;
    double gz = m->gz;

    double ax = m->ax;
    double ay = m->ay;
    double az = m->az;

    double beta = 0.1f;  // 2 * proportional gain (Kp)
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 =     _4q0*q2q2    + _2q2*ax +    _4q0*q1q1 - _2q1*ay;
        s1 =     _4q1*q3q3    - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2     + _2q0*ax +    _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3     - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);     // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q->w = q0;
    q->x = q1;
    q->y = q2;
    q->z = q3;
}

double radian2degrees(double radians) {
    return radians * (180.0f / M_PI);
}

double degress2radian(double degress) {
    return degress * (M_PI / 180.0f);
}

void quaternion_init(quaternion_t* q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void angle_init(angle_t* a) {
    a->z = 0.0f;
    a->y = 0.0f;
    a->x = 0.0f;
}

void angle_degress(angle_t* a) {
    a->z *= (180.0f / M_PI);
    a->y *= (180.0f / M_PI);
    a->x *= (180.0f / M_PI);
}

double sgn(double x) {
    return copysignf(1.f,x);
}

angle_t quaternion2xyz(quaternion_t* q) {
    angle_t a;

    double x = q->x;
    double y = q->y;
    double z = q->z;
    double w = q->w;

    double t0 = (x + z)*(x - z);     // x^2-z^2
    double t1 = (w + y)*(w - y);     // w^2-y^2

    double xx = 0.5f * (t0 + t1);    // 1/2 x of x'
    double xy = x*y + w*z;           // 1/2 y of x'
    double xz = w*y - x*z;           // 1/2 z of x'

    double t  = xx*xx + xy*xy;       // cos(theta)^2
    double yz = 2.0f * (y*z + w*x);  // z of y'

    a.z = atan2(xy, xx);             // yaw   (psi)
    a.y = atan(xz /sqrt(t));         // pitch (theta)

    if (t != 0) {
        a.x = atan2(yz, t1 - t0);
    } else {
        a.x = (2.0 * atan2(x, w) - sgn(xz) * a.z);
    }
    return a;
}


int main(void) {

    _delay(100);
    clock_setup();
    usart_setup();

    i2c_setup();
    systick_setup();

    printf("==== start ====\r\n");

    mpu_t mpu;
    mpu_setup(&mpu, I2C1, 0x68);
    _delay(10);
    mpu_calibrate(&mpu, 20000);
    printf("==== mpu ====\r\n");

    mpu_value_t mval;

    uint32_t prev_ts = 0, last_ts = 0;

    quaternion_t q;
    quaternion_init(&q);

    int i = 0;
    while (true) {
        mpu_read(&mpu, &mval);

        gpio_toggle(GPIOB, GPIO6);
        //if ((i % 350) == 0) {
        //    printf("gx=%8.4f gy=%8.4f gz=%8.4f ax=%8.4f ay=%8.4f az=%8.4f \r\n", mval.gx, mval.gy, mval.gz, mval.ax, mval.ay, mval.az);
        //}
        i++;
        last_ts = g_sys_tick_counter;
        float dt = (float)(last_ts - prev_ts) / (float)systic_freq;
        prev_ts = last_ts;

        ahrs_update(dt, &q, &mval);

        angle_t a = quaternion2xyz(&q);
        angle_degress(&a);

        printf("dt=%.6f y=%10.4f x=%10.4f z=%10.4f  \r\n", dt, a.y, a.x, a.z);

    };
}
