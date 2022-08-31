/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include "mutex.h"
#include "atomic.h"

void mutex_init(mutex_t* mutex) {
    mutex->value = 1;
}

int32_t mutex_wait(mutex_t* mutex) {
    //while(mutex->value <= 0);
    //mutex->value--;
    //return mutex->value;
    while (atomic_dec32(&(mutex->value), (int32_t)0) <= 0);
    return atomic_dec32(&(mutex->value), (int32_t)1);
}

int32_t mutex_post(mutex_t* mutex) {
    //mutex->value++;
    //return mutex->value;
    return atomic_inc32(&(mutex->value), (int32_t)1);
}
