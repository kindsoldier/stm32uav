/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#ifndef SBUS_H_QWERTY
#define SBUS_H_QWERTY

#define SBUS_FRAMESIZE 25
#define SBUS_CHANNELS  18
#define WRONG_RCVAL    -1

typedef struct {
    uint8_t  iframe[SBUS_FRAMESIZE];
    uint16_t rcvalue[SBUS_CHANNELS];
    int findex;
} sbus_t;

void sbus_init(sbus_t* bus);
bool sbus_recv(sbus_t* bus, uint8_t ibyte);
int sbus_chcount(sbus_t* bus);
int16_t sbus_getch(sbus_t* bus, int num);

#endif
