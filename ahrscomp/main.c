/*
 *
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

#define NS_MULTI (1000 * 1000 * 1000)

typedef struct timespec timespec_t;

typedef struct angle_s {
    double z;   // yaw
    double y;   // pitch
    double x;   // roll
} angle_t;


typedef struct quaternion_s {
    double w;
    double x;
    double y;
    double z;
} quaternion_t;


double radian2degrees(double radians) {
    return radians * (180.0f / M_PI);
}

double degress2radian(double degress) {
    return degress * (M_PI / 180.0f);
}

void quaternion_init(quaternion_t* q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void angle_init(angle_t* a) {
    a->z = 0.0f;
    a->y = 0.0f;
    a->x = 0.0f;
}

quaternion_t quaternion_multi(const quaternion_t* q1, const quaternion_t* q2) {
    quaternion_t r;
    r.x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
    r.y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
    r.z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;
    r.w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    return r;
}

void quaternion_norm(quaternion_t* q) {
    double len = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    q->w /= len;
    q->x /= len;
    q->y /= len;
    q->z /= len;
}

quaternion_t zyx2quaternion(angle_t* a) {

    quaternion_t q;

    double hy = 0.5f * a->z;
    double hp = 0.5f * a->y;
    double hr = 0.5f * a->x;

    double ys = sin(hy), yc = cos(hy);
    double ps = sin(hp), pc = cos(hp);
    double rs = sin(hr), rc = cos(hr);

    q.w = (rc*pc*yc + rs*ps*ys);
    q.x = (rs*pc*yc - rc*ps*ys);
    q.y = (rc*ps*yc + rs*pc*ys);
    q.z = (rc*pc*ys - rs*ps*yc);

    return q;

    // below here temp. hacks for error measures
    //if (q.w >= 0.0f) return;
    //quat_neg(q, q);
}

double sgn(double x) {
    return copysignf(1.f,x);
}

angle_t quaternion2xyz(quaternion_t* q) {
    angle_t a;

    double x = q->x;
    double y = q->y;
    double z = q->z;
    double w = q->w;

    double t0 = (x + z)*(x - z);     // x^2-z^2
    double t1 = (w + y)*(w - y);     // w^2-y^2

    double xx = 0.5f * (t0 + t1);    // 1/2 x of x'
    double xy = x*y + w*z;           // 1/2 y of x'
    double xz = w*y - x*z;           // 1/2 z of x'

    double t  = xx*xx + xy*xy;       // cos(theta)^2
    double yz = 2.0f * (y*z + w*x);  // z of y'

    a.z = atan2(xy, xx);             // yaw   (psi)
    a.y = atan(xz /sqrt(t));         // pitch (theta)

    if (t != 0) {
        a.x = atan2(yz, t1 - t0);
    } else {
        a.x = (2.0 * atan2(x, w) - sgn(xz) * a.z);
    }
    return a;
}

int main(int argc, char **argv) {

    quaternion_t q0;
    quaternion_init(&q0);
    printf("qw=%5.3f qx=%5.3f qy=%5.3f qz=%5.3f\n", q0.w, q0.x, q0.y, q0.z);

    angle_t a0;
    angle_init(&a0);
    a0.x = 0.01f;
    a0.y = 0.01f;
    a0.z = 0.01f;

    quaternion_t q1 = zyx2quaternion(&a0);
    printf("qw=%5.3f qx=%5.3f qy=%5.3f qz=%5.3f\n", q1.w, q1.x, q1.y, q1.z);

    quaternion_t q2 = q0;
    for (int i = 0; i < 100; i++) {
        q2 = quaternion_multi(&q1, &q2);
        quaternion_norm(&q2);
    }
    printf("qw=%5.3f qx=%5.3f qy=%5.3f qz=%5.3f\n", q2.w, q2.x, q2.y, q2.z);


    angle_t a1 = quaternion2xyz(&q2);

    printf("a.x=%.3f a.y=%.3f a.z=%.3f \n", a1.x, a1.y, a1.z);

    return 0;
}
