#include "morm-sp.h"

void morm_sp_task(void){

//print counter value each N seconds
loop:
	printf("MEM0: writes=%u, reads=%u\n", *M0_COUNTER_STORE, *M0_COUNTER_LOAD);
	printf("MEM1: writes=%u, reads=%u\n", *M1_COUNTER_STORE, *M1_COUNTER_LOAD);
	printf("MEM2: writes=%u, reads=%u\n", *M2_COUNTER_STORE, *M2_COUNTER_LOAD);
	printf("---\n");
	
	printf("CPU: iarith=%u, ilogical=%u\n",   *CPU_COUNTER_ARITH, *CPU_COUNTER_LOGICAL);
	printf("CPU: ishift=%u, ibranches=%u\n",  *CPU_COUNTER_SHIFT, *CPU_COUNTER_BRANCHES);
	printf("CPU: ijumps=%u, iloadstore=%u\n", *CPU_COUNTER_JUMPS, *CPU_COUNTER_LOADSTORE);
	printf("---\n");
	
	printf("ROUTER: active=%u\n", *ROUTER_COUNTER_ACTIVE);
	printf("---\n");
	
	goto loop;	
}
