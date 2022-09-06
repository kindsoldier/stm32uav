
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

const uint32_t systic_freq = 100 * 1000;

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
    //gpio_toggle(GPIOB, GPIO6);
}

uint32_t sys_tick_counter(void) {
    uint32_t val = g_sys_tick_counter;
    return val;
}

void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az) {

}

int main(void) {

    _delay(100);
    clock_setup();
    usart_setup();

    i2c_setup();
    systick_setup();

    mpu_t mpu;
    mpu_setup(&mpu, I2C1, 0x68);
    mpu_calibrate(&mpu, 5000);
    printf("==== start ====\r\n");

    mpu_value_t val;

    uint32_t old_time = 0;
    uint32_t new_time = 0;

    float freq = 100.0f;
    uint32_t delta = 0.0f;

    while (true) {
            mpu_read(&mpu, &val);
            gpio_toggle(GPIOB, GPIO6);

            ahrs_update(val.gx, val.gy, val.gz, val.ax, val.ay, val.az);

            printf("%12.3f\r\n", freq);

        new_time = g_sys_tick_counter;
        delta = (float)(old_time - new_time);
        if (delta != 0.0f) {
            freq = (float)systic_freq / (float)(new_time - old_time);
            //printf("%12.3f\r\n", freq);
        }
        old_time = new_time;
    };
}
