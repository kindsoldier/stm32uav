/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#ifndef FILTER_H_QWERTY
#define FILTER_H_QWERTY


typedef struct {
    double x0;
    double x1;
    double rc;
} lpf2_t;

void lpf2_init(lpf2_t *lpf, double freq);
double lpf2_apply(lpf2_t *lpf, double x, double dt);


typedef struct {
    double x1;
    double x2;
    double x3;
    double rc;
} lpf3_t;

void lpf3_init(lpf3_t *lpf, double freq);
double lpf3_apply(lpf3_t *lpf, double x, double dt);

#endif
