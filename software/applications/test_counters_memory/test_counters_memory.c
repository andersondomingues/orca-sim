#include "test_counters_memory.h"

#define MEM0_COUNTERS_STORE_ADDR 0x81000000
#define MEM0_COUNTERS_LOAD_ADDR  0x81000002
#define MEM1_COUNTERS_STORE_ADDR 0x81000004
#define MEM1_COUNTERS_LOAD_ADDR  0x81000006
#define MEM2_COUNTERS_STORE_ADDR 0x81000008
#define MEM2_COUNTERS_LOAD_ADDR  0x8100000A

#define WAIT_PERIOD 1

void test_counters_memory(void){

	//main memory counters
	volatile uint16_t* mem0_store_ptr = (uint16_t*) MEM0_COUNTERS_STORE_ADDR;
	volatile uint16_t* mem0_load_ptr  = (uint16_t*) MEM0_COUNTERS_LOAD_ADDR;
	
	//receive memory counters
	volatile uint16_t* mem1_store_ptr = (uint16_t*) MEM1_COUNTERS_STORE_ADDR;
	volatile uint16_t* mem1_load_ptr  = (uint16_t*) MEM1_COUNTERS_LOAD_ADDR;

	//send memory counters
	volatile uint16_t* mem2_store_ptr = (uint16_t*) MEM2_COUNTERS_STORE_ADDR;
	volatile uint16_t* mem2_load_ptr  = (uint16_t*) MEM2_COUNTERS_LOAD_ADDR;

	//print counter value each N seconds
	loop:
	
		printf("MEM0: reads=%d, writes=%d\n", *mem0_load_ptr, *mem0_store_ptr);
		printf("MEM1: reads=%d, writes=%d\n", *mem1_load_ptr, *mem1_store_ptr);
		printf("MEM2: reads=%d, writes=%d\n", *mem2_load_ptr, *mem2_store_ptr);
		printf("---\n");
		
		goto loop;

}
