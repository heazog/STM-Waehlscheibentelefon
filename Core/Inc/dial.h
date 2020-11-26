#ifndef DIAL_H
#define DIAL_H

#include <stdbool.h>
#include "stm32l4xx_hal.h"

#define MAX_DIGITS 15
#define MIN_DIGITS 8
#define WAIT_UNTIL_CALL 2000

void dial_start(void);

void dial_stop(void);

void dial_count(void);

bool dial_getNumber(char * number);

#endif
