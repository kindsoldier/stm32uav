/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#include <stdbool.h>

double mapval(double imin, double imax, double omin, double omax, double in, bool inv) {
    double odiap = (omax - omin);
    double k = 1.0 / (imax - imin);
    double out = 0.0;
    if (inv) {
        out = (omax - odiap/2.0) - in * k * odiap;
    } else {
        out = in * k * odiap + (odiap/2.0 + omin);
    }

    return out;
}
