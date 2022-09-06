/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include <libopencm3/stm32/usart.h>

#include <stdlib.h>
#include <stdio.h>

void usart_puts(uint32_t usart, uint8_t * str) {
    uint16_t i = 0;
    while (str[i] != 0) {
        usart_send_blocking(usart, str[i++]);
    }
}

void usart_putc(uint32_t usart, uint8_t c) {
    usart_send_blocking(usart, c);
}
