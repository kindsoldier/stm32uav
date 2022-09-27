/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */

#ifndef GEOMETRY_H_QWERTY
#define GEOMETRY_H_QWERTY

typedef struct {
    union { double z; double yaw; };
    union { double y; double pitch; };
    union { double x; double roll; };
} eulerangle_t;


typedef struct quaternion_s {
    union { double w; double q0; };
    union { double x; double q1; };
    union { double y; double q2; };
    union { double z; double q3; };
} quaternion_t;


typedef struct {
    double ax;
    double ay;
    double az;
    double gx;
    double gy;
    double gz;
} imuvec_t;

void eulerangle_init(eulerangle_t* a);
void eulerangle_norm(eulerangle_t* a);
void eulerangle_todegress(eulerangle_t* a);
void eulerangle_toradians(eulerangle_t* a);

void quaternion_init(quaternion_t* q);
void quaternion_toeuler(quaternion_t* q, eulerangle_t* a);

void quaternion_madgwick(quaternion_t* q, imuvec_t* m, double dt);

#endif
