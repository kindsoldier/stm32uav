/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#include <stdio.h>
#include <stdint.h>

#include <mixer.h>

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
        int oidx = mix->r[ridx].o;
        if (oidx < 0) continue;
        if (mix->o[oidx] == NULL) continue;
        *(mix->o[oidx]) = 0.0;
    }

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
