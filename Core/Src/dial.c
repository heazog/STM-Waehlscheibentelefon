#include "dial.h"
#include <string.h>

static uint32_t cnt = 0;
static char digits[MAX_DIGITS] = {0};
static uint8_t digitCnt = 0;
static uint32_t lastTime = 0;

void dial_start(void){
	cnt = 0;
}

void dial_stop(void){
	if(cnt == 10){
		digits[digitCnt] = '0';
	}else{
		digits[digitCnt] = '0' + cnt;
	}
	
	if(digitCnt == MAX_DIGITS || cnt > 10){
		// error -> reset
		digitCnt = 0;
		cnt = 0;
	}else{
		digitCnt++;
	}
}


void dial_count(void){
	cnt++;
	lastTime = HAL_GetTick();
}


bool dial_getNumber(char * number){
	if((HAL_GetTick()-lastTime) >= WAIT_UNTIL_CALL){
		if(digitCnt >= MIN_DIGITS){
			digits[digitCnt] = 0x00;
			strcpy(number, digits);
			digitCnt = 0;
			cnt = 0;
			return true;
		}
	}
	return false;
}
