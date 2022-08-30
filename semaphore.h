/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdint.h>

typedef struct {
    int32_t value;
} sem_t;

void sem_init(sem_t* sem, int32_t value);
int32_t sem_wait(sem_t* sem);
int32_t sem_post(sem_t* sem);
