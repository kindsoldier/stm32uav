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
    double w;
    double x;
    double y;
    double z;
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


#endif
