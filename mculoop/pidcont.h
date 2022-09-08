/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 *
 */


#ifndef PIDCONT_H_QWERTY
#define PIDCONT_H_QWERTY


typedef struct {
    double perror;
    double integ;

    double kp;
    double ki;
    double kd;
} pidcont_t;


void pidcont_init(pidcont_t* p);
void pidcont_setup(pidcont_t* p, double kp, double ki, double kd);
double pidcont_next(pidcont_t* p, double target, double actual, double dt);

#endif
