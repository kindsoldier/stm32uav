/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 *
 */


#include <stdint.h>
#include <pidcont.h>

void pidcont_init(pidcont_t* p) {
    p->perror = 0.0;
    p->integ = 0.0;

    p->kp = 0.0;
    p->ki = 0.0;
    p->kd = 0.0;
}


void pidcont_setup(pidcont_t* p, double kp, double ki, double kd) {
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}


double pidcont_next(pidcont_t* p, double target, double actual, double dt) {

    double error  = 0.0;
    double deriv  = 0.0;
    double integ  = 0.0;
    double output = 0.0;

    error = target - actual;
    integ += error * dt;

    deriv = error - p->perror;

    output = (p->kp * error) + (p->ki * integ) + (p->kd * deriv);

    p->perror = error;
    p->integ  = integ;
    return output;
}
