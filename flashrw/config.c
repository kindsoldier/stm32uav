/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <libopencm3/stm32/flash.h>

#include <config.h>
#include <stdint.h>

void config_init(config_t* c) {
    c->gz = 0x00;
    c->gy = 0x00;
};

config_t* config_getptr() {
    return (config_t*)&_config;
}

void config_save(config_t* ptr) {
    flash_unlock();
    flash_erase_sector(SECTOR_NO, sizeof(config_t));
    uint32_t caddr = (uint32_t)&_config;
    flash_program(caddr, (uint8_t*)ptr, (uint32_t)sizeof(config_t));
}
