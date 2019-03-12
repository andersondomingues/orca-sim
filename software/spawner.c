#include "applications/noc_test4/noc_test4.h"

void app_main(void)
{

	hf_spawn(receiver, 0, 0, 0, "receiver", 4096);		
	
	switch(hf_cpuid()){
		case 5:
		case 15:
			hf_spawn(sender, 0, 0, 0, "sender", 4096);			
			break;
		default:
			hf_spawn(receiver, 0, 0, 0, "receiver", 4096);
			break;
	}
}
