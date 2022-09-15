/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */


#ifndef IBUS_H_QWERTY
#define IBUS_H_QWERTY

#define IBUS_FRAMESIZE 32
#define IBUS_CHANNELS  14
#define WRONG_RCVAL    -1

typedef struct {
    uint8_t  iframe[IBUS_FRAMESIZE];
    uint16_t rcvalue[IBUS_CHANNELS];
    int findex;
} ibus_t;

void ibus_init(ibus_t* bus);
bool ibus_recv(ibus_t* bus, uint8_t ibyte);
int ibus_chcount(ibus_t* bus);
int16_t ibus_getch(ibus_t* bus, int num);

#endif
