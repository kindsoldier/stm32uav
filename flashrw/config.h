/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#ifndef CONFIG_H_QWERTY
#define CONFIG_H_QWERTY


#include <stdint.h>

#define SECTOR_NO 1
extern uint8_t _config;

typedef struct __attribute__ ((packed))  {
    int32_t gz;
    int32_t gy;
} config_t;

void config_init(config_t* c);
void config_save(config_t* ptr);
config_t* config_getptr();


#endif
