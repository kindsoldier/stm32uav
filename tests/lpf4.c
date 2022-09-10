/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <complex.h>

typedef struct {
    double x1;
    double x2;
    double x3;
    double rc;
} lpf_t;


void lpf_init(lpf_t *lpf, double freq) {
    lpf->x1 = 0.0;
    lpf->x2 = 0.0;
    lpf->x3 = 0.0;

    double order = 3.0;
    double c = 1.0 / sqrt(powf(2.0, 1.0 / order) - 1.0);
    lpf->rc = 1.0 / (2.0 * c * M_PI * freq);
}

double lpf_apply(lpf_t *lpf, double x0, double dt) {

    double k = dt / (lpf->rc + dt);

    lpf->x1 = lpf->x1 + k * (x0 - lpf->x1);
    lpf->x2 = lpf->x2 + k * (lpf->x1 - lpf->x2);
    lpf->x3 = lpf->x3 + k * (lpf->x2 - lpf->x3);
    return lpf->x3;
}



void dft(double* x1, double* y1, int m, int dir) {
    long i, k;
    double arg;
    double cosarg, sinarg;

    double x2[m];
    double y2[m];

    for (i = 0; i < m; i++) {
        x2[i] = 0;
        y2[i] = 0;
        arg = -dir * 2.0 * M_PI * (double)i / (double)m;
        for (k = 0; k < m; k++) {
            cosarg = cos(k * arg);
            sinarg = sin(k * arg);
            x2[i] += (x1[k] * cosarg - y1[k] * sinarg);
            y2[i] += (x1[k] * sinarg + y1[k] * cosarg);
        }
    }
    if (dir == 1) {
        for (i = 0; i < m; i++) {
            x1[i] = x2[i] / (double)m;
            y1[i] = y2[i] / (double)m;
        }
    } else {
        for (i = 0; i < m; i++) {
            x1[i] = x2[i];
            y1[i] = y2[i];
        }
    }
}


int main(int argc, char** argv) {

    double t = 0;
    double s = 0;
    double freq0 = 20;
    double dt = 0.001;

    lpf_t lpf;

    lpf_init(&lpf, 20.0);

    int c = 1024 * 2;
    double xa[c];
    double xb[c];
    double y[c];

    for (int i = 0; i < c; i++) {
        t = i * dt;
        for (int n = 0; n < 200; n += 20) {
            double freq = freq0 + n;
            s += cos(2 * M_PI * freq * t);
        }

        xa[i] = s;
        xb[i] = lpf_apply(&lpf, s, dt);;
    }

    for (int i = 0; i < c; i++) {
        y[i] = 0;
    }

    double rate = 1.0 / dt;


    FILE* fd;

    fd = popen("gnuplot -persistent", "w");

    fprintf(fd, "set terminal png size 1600,1200\n");
    fprintf(fd, "set output \"gensin4a.png\"\n");
    fprintf(fd, "plot '-' with lines\n");

    dft(xa, y, c, -1);
    for (int i = 0; i < c; i++) {
        double freq = rate * (double)i / (double)c;
        double amp = sqrt(xa[i] * xa[i] + y[i] * y[i]);

        if (freq < rate / 2) {
            fprintf(fd, "%e %e\n", freq, amp);
        }
    }
    fflush(fd);
    fprintf(fd, "e");
    fclose(fd);

    for (int i = 0; i < c; i++) {
        y[i] = 0;
    }


    fd = popen("gnuplot -persistent", "w");

    fprintf(fd, "set terminal png size 1600,1200\n");
    fprintf(fd, "set output \"gensin4b.png\"\n");
    fprintf(fd, "plot '-' with lines\n");

    dft(xb, y, c, -1);
    for (int i = 0; i < c; i++) {
        double freq = rate * (double)i / (double)c;
        double amp = sqrt(xb[i] * xb[i] + y[i] * y[i]);

        if (freq < rate / 2) {
            fprintf(fd, "%e %e\n", freq, amp);
        }
    }
    fflush(fd);
    fprintf(fd, "e");
    fclose(fd);

    return 0;
}
