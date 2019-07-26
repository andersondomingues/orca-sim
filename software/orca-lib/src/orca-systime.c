#include "orca-lib.h"
#include "orca-systime.h"

volatile uint32_t* SYSTIME_ADDR = (volatile uint32_t*) 0x80000014;

uint32_t GetHostTime(){
	return *SYSTIME_ADDR;
}