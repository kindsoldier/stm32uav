/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#ifndef FILTER_H_QWERTY
#define FILTER_H_QWERTY


typedef struct {
    double x0;
    double x1;
    double rc;
} lpf_t;

void lpf_init(lpf_t *lpf, double freq);
double lpf_apply(lpf_t *lpf, double x2, double dt);

#endif
