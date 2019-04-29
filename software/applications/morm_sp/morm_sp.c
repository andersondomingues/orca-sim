#include "morm_sp.h"

void morm_sp_task(void){

//main memory counters
volatile uint32_t* mem0_store_ptr =  MEM0_COUNTERS_STORE_ADDR;
volatile uint32_t* mem0_load_ptr  =  MEM0_COUNTERS_LOAD_ADDR;

//receive memory counters
volatile uint32_t* mem1_store_ptr =  MEM1_COUNTERS_STORE_ADDR;
volatile uint32_t* mem1_load_ptr  =  MEM1_COUNTERS_LOAD_ADDR;

//send memory counters
volatile uint32_t* mem2_store_ptr =  MEM2_COUNTERS_STORE_ADDR;
volatile uint32_t* mem2_load_ptr  =  MEM2_COUNTERS_LOAD_ADDR;

//cpu counters
volatile uint32_t* cpu_iarith     = CPU_COUNTERS_IARITH_ADDR;
volatile uint32_t* cpu_ilogical   = CPU_COUNTERS_ILOGICAL_ADDR;
volatile uint32_t* cpu_ishift     = CPU_COUNTERS_ISHIFT_ADDR;
volatile uint32_t* cpu_ibranches  = CPU_COUNTERS_IBRANCHES_ADDR;
volatile uint32_t* cpu_ijumps     = CPU_COUNTERS_IJUMPS_ADDR;
volatile uint32_t* cpu_iloadstore = CPU_COUNTERS_ILOADSTORE_ADDR;

//router
volatile uint32_t* router_active = ROUTER_COUNTERS_ACTIVE_ADDR;

//print counter value each N seconds
loop:
	printf("MEM0: writes=%u, reads=%u\n", *mem0_store_ptr, *mem0_load_ptr);
	printf("MEM1: writes=%u, reads=%u\n", *mem1_store_ptr, *mem1_load_ptr);
	printf("MEM2: writes=%u, reads=%u\n", *mem2_store_ptr, *mem2_load_ptr);
	printf("---\n");
	
	printf("CPU: iarith=%u, ilogical=%u\n", *cpu_iarith, *cpu_ilogical);
	printf("CPU: ishift=%u, ibranches=%u\n", *cpu_ishift, *cpu_ibranches);
	printf("CPU: ijumps=%u, iloadstore=%u\n", *cpu_ijumps, *cpu_iloadstore);
	printf("---\n");
	
	printf("ROUTER: active=%u\n", *router_active);
	printf("---\n");
	
	goto loop;
	
}
