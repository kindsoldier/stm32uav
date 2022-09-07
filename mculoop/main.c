
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

#include <usartu.h>
#include <mpu6050.h>
#include <geometry.h>
#include <madgwick.h>


const uint32_t g_systick_freq = 100 * 1000;
uint32_t g_sys_tick_counter;

//static void _delay(uint32_t n) {
//    for (int i = 0; i < n * ; i++)
//        __asm__("nop");
//}

static void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_I2C1);

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

static void systick_setup(uint32_t systic_freq) {
    g_sys_tick_counter = 0;

    //gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    //gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6);

    systick_set_frequency(systic_freq, rcc_ahb_frequency);
    systick_interrupt_enable();
    systick_counter_enable();
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

void sys_tick_handler(void) {
    g_sys_tick_counter++;
    //gpio_toggle(GPIOB, GPIO6);
}

uint32_t sys_tick_counter(void) {
    uint32_t val = g_sys_tick_counter;
    return val;
}

int main(void) {

    clock_setup();
    usart_setup(USART1, GPIOA, GPIO9 | GPIO10, 460800);
    i2c_setup(I2C1, GPIOB, GPIO8 | GPIO9);
    systick_setup(g_systick_freq);

    imu_t imu;
    printf("==== imu initialize ====\r\n");
    imu_setup(&imu, I2C1, 0x68);
    imu_calibrate(&imu, 20000);
    printf("==== imu started ====\r\n");

    imuvec_t mval;
    quaternion_t q;
    quaternion_init(&q);

    uint32_t prev_ts = 0;
    uint32_t last_ts = 0;

    printf("==== main loop ====\r\n");

    while (true) {
        imu_getvec(&imu, &mval);

        last_ts = g_sys_tick_counter;
        double dt = (float)(last_ts - prev_ts) / (float)g_systick_freq;
        prev_ts = last_ts;

        madgwick(dt, &q, &mval);

        eulerangle_t a;
        quaternion_toeuler(&q, &a);
        eulerangle_todegress(&a);

        printf("dt=%.6f y=%10.4f x=%10.4f z=%10.4f  \r\n", dt, a.y, a.x, a.z);

        //double b2 = 0.0;
        //double K = 0.01;
        //b2 = b2*(1 - K) + b*K;
    };
}
