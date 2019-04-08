#ifndef FIX_HF_INCLUDES
#define FIX_HF_INCLUDES
//#include <hellfire.h>
//#include <noc.h>
#endif

//application-specific includes 
#include "../applications/test_counters_memory/test_counters_memory.h"

void app_main(void)
{
	//if(hf_cpuid() == 2) hf_spawn(mapper_listener, 0, 0, 0, "mapper", 4096);
	hf_spawn(test_counters_memory, 0, 0, 0, "test_counters", 4096);
}
