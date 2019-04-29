//basic resources
#include "orca-lib.h"

//extended functionalities
#include "orca-hardware-counters.h"

//application-specific code
//IMPORT_APP("morm_sp")
#include "../../applications/morm_sp/morm_sp.h"

//applications' entry-point (sort of)
void app_main(void)
{
	switch(hf_cpuid()){
		case 2:
		case 3:
		default:
			hf_spawn(morm_sp_task, 0, 0, 0, "morm_sp_task", 4096);			
			break;
	}
}
