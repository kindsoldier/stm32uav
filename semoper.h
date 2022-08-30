/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#ifndef SEMOPER_H_QWERTY
#define SEMOPER_H_QWERTY

#include <stdint.h>

int32_t atom_inc32(volatile int32_t *addr, int32_t value);
int32_t atom_dec32(volatile int32_t *addr, int32_t value);

#endif
