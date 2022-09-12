/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdio.h>
#include <math.h>

#include <geometry.h>

void madgwick(double dt, quaternion_t* q, imuvec_t* m) {

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
