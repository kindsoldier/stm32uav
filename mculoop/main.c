
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
#include <libopencm3/stm32/timer.h>


#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <usartu.h>
#include <mpu6050.h>
#include <geometry.h>
#include <madgwick.h>
#include <pidcont.h>
#include <filter.h>
#include <misc.h>



const uint32_t g_systick_freq = 50 * 1000;
uint32_t g_sys_tick_counter;

static void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM5);
}

static void usart_setup(uint32_t usart, uint32_t gpioport, uint32_t gpiopins, uint32_t baudrate) {
    usart_disable(usart);

    gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, gpiopins);
    gpio_set_af(gpioport, GPIO_AF7, gpiopins);
    gpio_set_output_options(gpioport, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, gpiopins);

    usart_set_baudrate(usart, baudrate);
    usart_set_databits(usart, 8);
    usart_set_stopbits(usart, USART_STOPBITS_1);
    usart_set_parity(usart, USART_PARITY_NONE);
    usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
    usart_set_mode(usart, USART_MODE_TX_RX);
    usart_disable_rx_interrupt(usart);

    usart_enable(usart);
}

static void i2c_setup(uint32_t i2c, uint32_t gpioport, uint32_t gpiopins) {
    gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_PULLUP, gpiopins);
    gpio_set_output_options(gpioport, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, gpiopins);
    gpio_set_af(gpioport, GPIO_AF4, gpiopins);

    i2c_reset(i2c);
    i2c_peripheral_disable(i2c);
    i2c_set_speed(i2c, i2c_speed_fm_400k, I2C_CR2_FREQ_36MHZ);
    i2c_peripheral_enable(i2c);
}


static void systick_setup(uint32_t systic_freq) {
    g_sys_tick_counter = 0;

    systick_set_frequency(systic_freq, rcc_ahb_frequency);
    systick_interrupt_enable();
    systick_counter_enable();
}


void sys_tick_handler(void) {
    g_sys_tick_counter++;
}

uint32_t sys_tick_counter(void) {
    uint32_t val = g_sys_tick_counter;
    return val;
}


#define PWM100 9999
#define PWM150 6666
#define PWM200 4999
#define PWM250 3999
#define PWM300 3333
#define PWM330 3030

static void timer_init(uint32_t timer) {

    int prescale = rcc_ahb_frequency / (2 * 1000 * 1000) - 1;
    int period = PWM330;

    timer_disable_counter(timer);
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_disable_preload(timer);
    timer_continuous_mode(timer);
    timer_set_prescaler(timer, prescale);
    timer_set_period(timer, period);
    timer_set_repetition_counter(timer, 0);
    timer_enable_break_main_output(timer);
    timer_enable_counter(timer);
}

static void tc_init(uint32_t timer, uint32_t channel) {

    timer_disable_oc_output(timer, channel);
    timer_set_oc_value(timer, channel, 0);
    timer_disable_oc_clear(timer, channel);
    timer_enable_oc_preload(timer, channel);
    timer_set_oc_slow_mode(timer, channel);
    timer_set_oc_mode(timer, channel, TIM_OCM_PWM1);

    timer_set_oc_polarity_high(timer, channel);
    timer_set_oc_idle_state_set(timer, channel);

    timer_set_oc_value(timer, channel, 20000);
    timer_enable_oc_output(timer, channel);
}

static void tc_setratio(uint32_t timer, uint32_t channel, uint32_t ratio) {
    uint32_t period = TIM_ARR(timer);
    uint32_t value = (period * ratio) / 1000;
    timer_set_oc_value(timer, channel, value);
}

static void timer_gpio_setup(uint32_t gpio_port, uint32_t gpio_af, uint32_t gpio_pin) {
    gpio_mode_setup(gpio_port, GPIO_MODE_AF, GPIO_PUPD_NONE, gpio_pin);
    gpio_set_af(gpio_port, gpio_af, gpio_pin);
    gpio_set_output_options(gpio_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, gpio_pin);
}


typedef struct {
    double start;
    double freq;
} systimer_t;

void systimer_init(systimer_t* timer, double freq, double start) {
    timer->start = start;
    timer->freq = freq;
}

double systimer_apply(systimer_t* timer, double timestamp) {
    double difftime = (timestamp - timer->start) / timer->freq;
    timer->start = timestamp;
    return difftime;
}


int main(void) {

    clock_setup();
    usart_setup(USART1, GPIOA, GPIO9 | GPIO10, 460800);

    i2c_setup(I2C1, GPIOB, GPIO8 | GPIO9);
    systick_setup(g_systick_freq);

    timer_gpio_setup(GPIOA, GPIO_AF1, GPIO0);
    timer_gpio_setup(GPIOA, GPIO_AF1, GPIO1);
    timer_gpio_setup(GPIOA, GPIO_AF1, GPIO2);
    timer_gpio_setup(GPIOA, GPIO_AF1, GPIO3);

    uint32_t timer = TIM2;
    timer_init(timer);
    tc_init(timer, TIM_OC1);
    tc_init(timer, TIM_OC2);
    tc_init(timer, TIM_OC3);
    tc_init(timer, TIM_OC4);

    tc_setratio(timer, TIM_OC1, 10);
    tc_setratio(timer, TIM_OC2, 30);
    tc_setratio(timer, TIM_OC3, 50);
    tc_setratio(timer, TIM_OC4, 70);

    imu_t imu;
    imu_setup(&imu, I2C1, 0x68);
    imu_calibrate(&imu, 100);

    printf("start\r\n");

    imuvec_t mval;
    quaternion_t q;
    quaternion_init(&q);

    double ak = 50.0;
    lpf3_t lpfax;
    lpf3_t lpfay;
    lpf3_t lpfaz;

    lpf3_init(&lpfax, ak);
    lpf3_init(&lpfay, ak);
    lpf3_init(&lpfaz, ak);

    lpf3_t lpfgx;
    lpf3_t lpfgy;
    lpf3_t lpfgz;

    lpf3_init(&lpfgx, ak);
    lpf3_init(&lpfgy, ak);
    lpf3_init(&lpfgz, ak);

    systimer_t systimer;
    systimer_init(&systimer, (double)g_systick_freq, (double)g_sys_tick_counter);

    pidcont_t p;
    pidcont_init(&p);

    double kp = 0;
    double ki = 4000.0;

    pidcont_setup(&p, kp, ki, 0);

    while (true) {
        imu_getvec(&imu, &mval);

        double dt = systimer_apply(&systimer, g_sys_tick_counter);

        mval.ax = lpf3_apply(&lpfax, mval.ax, dt);
        mval.ay = lpf3_apply(&lpfay, mval.ay, dt);
        mval.az = lpf3_apply(&lpfaz, mval.az, dt);

        mval.gx = lpf3_apply(&lpfgx, mval.gx, dt);
        mval.gy = lpf3_apply(&lpfgy, mval.gy, dt);
        mval.gz = lpf3_apply(&lpfgz, mval.gz, dt);

        madgwick(dt, &q, &mval);

        eulerangle_t a;
        quaternion_toeuler(&q, &a);
        eulerangle_todegress(&a);

        double pmin = -90.0;
        double pmax =  90.0;

        double omin =  150.0;
        double omax =  850.0;

        uint32_t out1 = (uint32_t)mapval(pmin, pmax, omin, omax, a.y, true);
        uint32_t out2 = (uint32_t)mapval(pmin, pmax, omin, omax, a.y, false);

        tc_setratio(timer, TIM_OC1, out1);
        tc_setratio(timer, TIM_OC2, out2);

        printf("dt=%.6f %lu <-pitch=%8.3f  roll=%8.3f  yaw=%8.3f\r\n", dt, out1, a.y, a.x, a.z);

        //double out = pidcont_apply(&p, 0, a.pitch, dt);
        //printf("dt=%.6f pitch=%8.3f  out=%10.3f %10.6f \r\n", dt, a.pitch, out, p.integ);
    };
}
