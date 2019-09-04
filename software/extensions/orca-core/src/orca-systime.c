#include "orca-core.h"
#include "orca-systime.h"

volatile uint32_t* SYSTIME_ADDR = (volatile uint32_t*) COMM_HOSTTIME;

uint32_t GetHostTime(){
	return *SYSTIME_ADDR;
}