#ifndef FONA_H
#define FONA_H

#include "main.h"

void fona_init(void);

void fona_voiceVolume(uint8_t vol);
void fona_ringVolume(uint8_t vol);
void fona_call(const char* number);
void fona_hangUp(void);
void fona_pickUp(void);
void fona_sms(const char* number, const char* msg);


#endif
