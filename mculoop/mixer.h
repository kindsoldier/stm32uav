/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#ifndef MIXER_H_QWERTY
#define MIXER_H_QWERTY

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


void mixer_init(mixer_t* mix);
void mixer_iset(mixer_t* mix, int n, double* i);
void mixer_oset(mixer_t* mix, int n, double* o);
void mixer_rset(mixer_t* mix, int n, int i, int o, double k);
void mixer_apply(mixer_t* mix);

#endif
