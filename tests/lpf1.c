/*
 *
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */


#include <stdio.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>



typedef struct {
    double x0;
    double x1;
    double rc;
} lpf_t;

void lpf_init(lpf_t *lpf, double freq) {
    lpf->x0 = 0.0;
    lpf->x1 = 0.0;

    double order = 1.5;
    double n = 1 / sqrt(pow(2, 1.0 / order) - 1);
    lpf->rc = 1 / (2 * n * M_PI * freq);
}


double lpf_next(lpf_t *lpf, double x, double dt) {

    double k = dt / (lpf->rc + dt);

    lpf->x1 = lpf->x1 + k * (x - lpf->x1);
    lpf->x0 = lpf->x0 + k * (lpf->x1 - lpf->x0);
    return lpf->x0;
}

int main(int argc, char **argv) {

    double t = 0;
    double s = 0;
    double freq1 = 10;
    double freq2 = 50;
    double freq3 = 200;
    double dt = 0.0001;
    double l = 0.5;

    int fd = open("freq.dat", O_WRONLY | O_TRUNC | O_CREAT, 0666);
    printf("start\n");

    if (fd < 0) { return 1; };

    lpf_t lpf;
    lpf_init(&lpf, 30.0);


    for (int i = 0; i < (l / dt); i++) {
        t = i * dt;
        double s1 = sin(2 * M_PI * freq1 * t);
        double s2 = cos(2 * M_PI * freq2 * t);
        double s3 = sin(2 * M_PI * freq3 * t);
        s = (s1 + s2 + s3) / 3.0;
        s = lpf_next(&lpf, s, dt);
        dprintf(fd, "%e\t%e\n", t, s);
    }
    fsync(fd);
    close(fd);
    return 0;
}
