#include "test_counters_memory.h"

#define MEM0_COUNTERS_STORE_ADDR (uint32_t*) 0x81000000
#define MEM0_COUNTERS_LOAD_ADDR  (uint32_t*) 0x81000004
#define MEM1_COUNTERS_STORE_ADDR (uint32_t*) 0x81000008
#define MEM1_COUNTERS_LOAD_ADDR  (uint32_t*) 0x8100000C
#define MEM2_COUNTERS_STORE_ADDR (uint32_t*) 0x81000010
#define MEM2_COUNTERS_LOAD_ADDR  (uint32_t*) 0x81000014

#define WAIT_PERIOD 1

void test_counters_memory(void){

	//main memory counters
	volatile uint32_t* mem0_store_ptr =  MEM0_COUNTERS_STORE_ADDR;
	volatile uint32_t* mem0_load_ptr  =  MEM0_COUNTERS_LOAD_ADDR;
	
	//receive memory counters
	volatile uint32_t* mem1_store_ptr =  MEM1_COUNTERS_STORE_ADDR;
	volatile uint32_t* mem1_load_ptr  =  MEM1_COUNTERS_LOAD_ADDR;

	//send memory counters
	volatile uint32_t* mem2_store_ptr =  MEM2_COUNTERS_STORE_ADDR;
	volatile uint32_t* mem2_load_ptr  =  MEM2_COUNTERS_LOAD_ADDR;

	//print counter value each N seconds
	loop:
		printf("MEM0: writes=%u, reads=%u\n", *mem0_store_ptr, *mem0_load_ptr, *mem0_load_ptr);
		printf("MEM1: writes=%u, reads=%u\n", *mem1_store_ptr, *mem1_load_ptr);
		printf("MEM2: writes=%u, reads=%u\n", *mem2_store_ptr, *mem2_load_ptr);
		printf("---\n");
		
		goto loop;

}
