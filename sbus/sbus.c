/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */



#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <sbus.h>

#define SBUS_STARTFR_BYTE 0x0F

void sbus_init(sbus_t* bus) {
    memset(bus->iframe, 0, sizeof(bus->iframe));
    memset(bus->rcvalue, 0, sizeof(bus->rcvalue));
    bus->findex = 0;
}


bool sbus_recv(sbus_t* bus, uint8_t ibyte) {

    printf("0x%02x ", ibyte);

    if(bus->findex == 0 && ibyte != SBUS_STARTFR_BYTE) {
        return false;
    }
    if(bus->findex < SBUS_FRAMESIZE) {
        bus->iframe[bus->findex] = ibyte;
        bus->findex++;

        //printf("0x%02x ", ibyte);

        return false;
    }

    if(bus->findex == SBUS_FRAMESIZE) {
        bus->findex = 0;
        //printf("0x%02x\r\n", bus->iframe[0]);
    }
    return false;
}

int sbus_chcount(sbus_t* bus) {
    return SBUS_CHANNELS;
}

int16_t sbus_getch(sbus_t* bus, int num) {
    if (num < SBUS_CHANNELS) {
        return (int16_t)bus->rcvalue[num];
    }
    return WRONG_RCVAL;
}
