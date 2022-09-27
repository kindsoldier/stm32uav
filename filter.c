/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <math.h>
#include <filter.h>

void lpf2_init(lpf2_t *lpf, double freq) {
    lpf->x0 = 0.0;
    lpf->x1 = 0.0;

    double order = 1.5;
    double n = 1 / sqrt(pow(2, 1.0 / order) - 1);
    lpf->rc = 1 / (2 * n * M_PI * freq);
}


double lpf2_apply(lpf2_t *lpf, double x2, double dt) {

    double k = dt / (lpf->rc + dt);

    lpf->x1 = lpf->x1 + k * (x2 - lpf->x1);
    lpf->x0 = lpf->x0 + k * (lpf->x1 - lpf->x0);
    return lpf->x0;
}


void lpf3_init(lpf3_t *lpf, double freq) {
    lpf->x1 = 0.0;
    lpf->x2 = 0.0;
    lpf->x3 = 0.0;

    double order = 2.0;
    double c = 1.0 / sqrt(powf(2.0, 1.0 / order) - 1.0);
    lpf->rc = 1.0 / (2.0 * c * M_PI * freq);
}

double lpf3_apply(lpf3_t *lpf, double x0, double dt) {

    double k = dt / (lpf->rc + dt);

    lpf->x1 = lpf->x1 + k * (x0 - lpf->x1);
    lpf->x2 = lpf->x2 + k * (lpf->x1 - lpf->x2);
    lpf->x3 = lpf->x3 + k * (lpf->x2 - lpf->x3);
    return lpf->x3;
}
