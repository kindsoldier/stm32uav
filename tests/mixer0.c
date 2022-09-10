/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdio.h>
#include <stdint.h>


#define ICOUNT 16
#define OCOUNT 16
#define RCOUNT 16


typedef struct {
    int i;
    int o;
    double k;
} rule_t;

typedef struct {
    double* i[ICOUNT];
    double* o[OCOUNT];

    rule_t r[RCOUNT];
} mixer_t;


void mixer_init(mixer_t* mix) {
    for (int i = 0; i < ICOUNT; i++) {
        mix->i[i] = NULL;
    }
    for (int i = 0; i < OCOUNT; i++) {
        mix->o[i] = NULL;
    }
    for (int i = 0; i < RCOUNT; i++) {
        mix->r[i].i = -1;
        mix->r[i].o = -1;
        mix->r[i].k = 0.0;
    }
}

void mixer_iset(mixer_t* mix, int n, double* i) {
    mix->i[n] = i;
}

void mixer_oset(mixer_t* mix, int n, double* o) {
    mix->o[n] = o;
}

void mixer_rset(mixer_t* mix, int n, int i, int o, double k) {
    mix->r[n].i = i;
    mix->r[n].o = o;
    mix->r[n].k = k;
}

void mixer_apply(mixer_t* mix) {
    for (int ridx = 0; ridx < RCOUNT; ridx++) {

        int iidx = mix->r[ridx].i;
        int oidx = mix->r[ridx].o;

        if (iidx < 0) continue;
        if (oidx < 0) continue;

        if (mix->i[iidx] == NULL) continue;
        if (mix->o[oidx] == NULL) continue;


        *(mix->o[oidx]) += *(mix->i[iidx]) * mix->r[ridx].k;
    }
}


int main(int argc, char **argv) {

    mixer_t mix;
    mixer_init(&mix);

    double ix =  10.0;
    double iy =  -15.0;

    double ox = 0.0;
    double oy = 0.0;

    mixer_iset(&mix, 0, &ix);
    mixer_iset(&mix, 1, &iy);

    mixer_oset(&mix, 0, &ox);
    mixer_oset(&mix, 1, &oy);

    mixer_rset(&mix, 0, 0, 0,  0.5);
    mixer_rset(&mix, 1, 1, 1,  0.5);
    mixer_rset(&mix, 2, 0, 1,  0.5);
    mixer_rset(&mix, 3, 1, 0, -0.5);


    mixer_apply(&mix);

    printf("ox=%10.3f\n", ox);
    printf("yx=%10.3f\n", oy);

    return 0;
}
