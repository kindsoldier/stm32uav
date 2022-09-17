/*
 * Copyright 2022 Oleg Borodin  <borodin@unix7.org>
 */



#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <ibus.h>

#define STARTFR_BYTE 0x20
#define COMMAND_BYTE 0x40

void ibus_init(ibus_t* bus) {
    memset(bus->iframe, 0, sizeof(bus->iframe));
    memset(bus->rcvalue, 0, sizeof(bus->rcvalue));
    bus->findex = 0;
}


bool ibus_recv(ibus_t* bus, uint8_t ibyte) {

    if(bus->findex == 0 && ibyte != STARTFR_BYTE) {
        return false;
    }
    if(bus->findex == 1 && ibyte != COMMAND_BYTE) {
        bus->findex = 0;
        return false;
    }
    if(bus->findex < IBUS_FRAMESIZE) {
        bus->iframe[bus->findex++] = ibyte;
        return false;
    }

    if(bus->findex == IBUS_FRAMESIZE) {
        bus->findex = 0;
        uint16_t chksum = 0xFFFF;
        for (int i = 0; i < 30; i++) {
            chksum -= bus->iframe[i];
        }
        uint16_t rxsum = bus->iframe[30] + (bus->iframe[31] << 8);
        if (chksum == rxsum) {
            for (int c = 0; c < IBUS_CHANNELS; c++) {
                int offset = c * 2 + 2;
                uint16_t hival = ((bus->iframe[offset + 1] & 0x0F) << 8);
                uint16_t loval = bus->iframe[offset] & 0xFF;
                bus->rcvalue[c] = hival + loval;
            }
            return true;
        }
    }
    return false;
}

int ibus_chcount(ibus_t* bus) {
    return IBUS_CHANNELS;
}

int16_t ibus_getch(ibus_t* bus, int num) {
    if (num < IBUS_CHANNELS) {
        return (int16_t)bus->rcvalue[num];
    }
    return WRONG_RCVAL;
}
