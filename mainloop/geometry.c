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
    q->w = 1.0;
    q->x = 0.0;
    q->y = 0.0;
    q->z = 0.0;
}

void quaternion_toeuler(quaternion_t* q, eulerangle_t* a) {

    double x = q->x;
    double y = q->y;
    double z = q->z;
    double w = q->w;

    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;

    double t0 = (x + z)*(x - z);     // x^2-z^2
    double t1 = (w + y)*(w - y);     // w^2-y^2

    double n1 = 0.5 * (t0 + t1);     // 1/2 x of x'
    double n2 = x*y + w*z;           // 1/2 y of x'
    double n3 = w*y - x*z;           // 1/2 z of x'

    double t  = n1*n1 + n2*n2;       // cos(theta)^2
    double n4 = 2.0 * (y*z + w*x);   // z of y'

    az = atan2(n2, n1);              // yaw   (psi)
    ay = atan(n3 / sqrt(t));         // pitch (theta)

    if (t != 0.0) {                  // roll
        ax = atan2(n4, t1 - t0);
    } else {
        ax = (2.0 * atan2(x, w) - copysign(1.0, n3) * az);
    }

    a->x = ax;
    a->y = ay;
    a->z = az;
}


void quaternion_madgwick(quaternion_t* q, imuvec_t* m, double dt) {

    double q0 = q->w;
    double q1 = q->x;
    double q2 = q->y;
    double q3 = q->z;      // quaternion of sensor frame relative to auxiliary frame

    double gx = m->gx;
    double gy = m->gy;
    double gz = m->gz;

    double ax = m->ax;
    double ay = m->ay;
    double az = m->az;

    double beta = 0.1f;  // 2 * proportional gain (Kp)
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5 * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5 * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5 * ( q0*gz + q1*gy - q2*gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

        // Normalise accelerometer measurement
        recipNorm = 1.0 / sqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Gradient decent algorithm corrective step
        s0 = 4.0*q0*q2*q2 + 2.0*q2*ax + 4.0*q0*q1*q1 - 2.0*q1*ay;
        s1 = 4.0*q1*q3*q3 - 2.0*q3*ax + 4.0*q0*q0*q1 - 2.0*q0*ay - 4.0*q1 + 8.0*q1*q1*q1 + 8.0*q1*q2*q2 + 4.0*q1*az;
        s2 = 4.0*q0*q0*q2 + 2.0*q0*ax + 4.0*q2*q3*q3 - 2.0*q3*ay - 4.0*q2 + 8.0*q2*q1*q1 + 8.0*q2*q2*q2 + 4.0*q2*az;
        s3 = 4.0*q1*q1*q3 - 2.0*q1*ax + 4.0*q2*q2*q3 - 2.0*q2*ay;

        // Normalise step magnitude
        recipNorm = 1.0 / sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = 1.0 / sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q->w = q0;
    q->x = q1;
    q->y = q2;
    q->z = q3;
}
