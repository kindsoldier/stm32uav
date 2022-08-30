/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


typedef struct {
    int value;
} sem_t;

int sem_init(sem_t* sem, int value);
int sem_wait(sem_t* sem);
int sem_post(sem_t* sem);
