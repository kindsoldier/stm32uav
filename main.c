/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "scheduler.h"
#include "usartu.h"
#include "semoper.h"


void delay(uint32_t n) {
    for (volatile int i = 0; i < n * 800; i++)
        __asm__("nop");
}

static void clock_setup(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup(void) {

    usart_disable(USART1);
    nvic_disable_irq(NVIC_USART1_IRQ);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    usart_disable_rx_interrupt(USART1);

    usart_enable(USART1);
}

static void systick_setup(void) {
    systick_set_frequency(10000, rcc_ahb_frequency);
    systick_interrupt_enable();
    systick_counter_enable();
}

int g_uptime;

void task1(void) {
    while (true) {
        printf("task 1 %d\r\n", g_uptime);
        delay(3000);
    };
}

void task2(void) {
    while (true) {
        printf("task 2 %d\r\n", g_uptime);
        delay(3100);
    };
}

void task3(void) {
    while (true) {
        printf("task 3 %d\r\n", g_uptime);
        delay(3200);
    };
}

void task4(void) {
    static volatile int32_t t4;

    while (true) {
        atom_inc32(&t4, (int32_t)1);
        printf("task 4 %d %lu\r\n", g_uptime, t4);
        delay(3300);
    };
}

void sys_tick_handler(void) {
    g_uptime++;
    scheduler_yield();
}


int main(void) {
    g_uptime = 0;

    static scheduler_t g_scheduler;

    clock_setup();
    usart_setup();
    scheduler_init(&g_scheduler);
    scheduler_task(&g_scheduler, 0, task1);
    scheduler_task(&g_scheduler, 1, task2);
    scheduler_task(&g_scheduler, 2, task3);
    scheduler_task(&g_scheduler, 3, task4);

    systick_setup();

    while (true);
}
