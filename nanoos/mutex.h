/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdint.h>

typedef struct {
    int32_t value;
} mutex_t;

void mutex_init(mutex_t* mutex);
int32_t mutex_lock(mutex_t* mutex);
int32_t mutex_unlock(mutex_t* mutex);
