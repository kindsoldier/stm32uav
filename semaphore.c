/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include "semaphore.h"

typedef struct {
    int value;
} sem_t;

int sem_init(sem_t* sem, int value) {
    sem->value = value;
}

int sem_wait(sem_t* sem) {
    while(sem->value > 0);
    sem->value--;
}


int sem_post(sem_t* sem) {
    sem->value++;
}
