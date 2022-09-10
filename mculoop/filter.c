/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <math.h>
#include <filter.h>

void lpf_init(lpf_t *lpf, double freq) {
    lpf->x0 = 0.0;
    lpf->x1 = 0.0;

    double order = 1.5;
    double n = 1 / sqrt(pow(2, 1.0 / order) - 1);
    lpf->rc = 1 / (2 * n * M_PI * freq);
}


double lpf_apply(lpf_t *lpf, double x2, double dt) {

    double k = dt / (lpf->rc + dt);

    lpf->x1 = lpf->x1 + k * (x2 - lpf->x1);
    lpf->x0 = lpf->x0 + k * (lpf->x1 - lpf->x0);
    return lpf->x0;
}
