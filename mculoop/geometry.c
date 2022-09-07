/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdio.h>
#include <math.h>
#include <geometry.h>

void eulerangle_init(eulerangle_t* a) {
    a->z = 0.0;
    a->y = 0.0;
    a->x = 0.0;
}

void eulerangle_todegress(eulerangle_t* a) {
    a->z *= (180.0 / M_PI);
    a->y *= (180.0 / M_PI);
    a->x *= (180.0 / M_PI);
}

void eulerangle_toradians(eulerangle_t* a) {
    a->z *= (M_PI / 180.0);
    a->y *= (M_PI / 180.0);
    a->x *= (M_PI / 180.0);
}


void eulerangle_norm(eulerangle_t* a) {
    double n = sqrt(a->x*a->x + a->y*a->y + a->z*a->z);
    a->x *= n;
    a->y *= n;
    a->z *= n;
}

void quaternion_init(quaternion_t* q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void quaternion_toeuler(quaternion_t* q, eulerangle_t* a) {

    double x = q->x;
    double y = q->y;
    double z = q->z;
    double w = q->w;

    double t0 = (x + z)*(x - z);     // x^2-z^2
    double t1 = (w + y)*(w - y);     // w^2-y^2

    double xx = 0.5 * (t0 + t1);     // 1/2 x of x'
    double xy = x*y + w*z;           // 1/2 y of x'
    double xz = w*y - x*z;           // 1/2 z of x'

    double t  = xx*xx + xy*xy;       // cos(theta)^2
    double yz = 2.0 * (y*z + w*x);   // z of y'

    a->z = atan2(xy, xx);             // yaw   (psi)
    a->y = atan(xz /sqrt(t));         // pitch (theta)

    if (t != 0) {
        a->x = atan2(yz, t1 - t0);
    } else {
        a->x = (2.0*atan2(x, w) - copysign(1.0, xz)*a->z);
    }
}
