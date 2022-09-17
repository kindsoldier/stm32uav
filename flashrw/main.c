
/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <usartu.h>

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

#define SECTOR_NO 1
extern uint8_t _config;


typedef struct {
    int32_t gz;
    int32_t gy;
} config_t;

void config_init(config_t* c){
    c->gz = 0x12;
    c->gy = 0x17;
};

int main(void) {

    clock_setup();
    usart_setup(USART1, GPIOA, GPIO9 | GPIO10, 115200);

    uint32_t caddr = 0xFFFFFFFF;
    caddr = (uint32_t)&_config;
    config_t* dconfig = (config_t*)caddr;

    config_t tconfig;
    config_init(&tconfig);

    flash_unlock();
    flash_erase_sector(SECTOR_NO, sizeof(config_t));

    printf("\r\n");
    printf("gy = 0x%08lx \r\n", dconfig->gy);

    flash_program(caddr, (uint8_t*)&tconfig, (uint32_t)sizeof(config_t));

    printf("gy = 0x%08lx \r\n", dconfig->gy);

    while (true) {

    };
}
