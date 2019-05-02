//basic resources
#include "orca-lib.h"

//extended functionalities
#include "orca-hardware-counters.h"

//application-specific code
//IMPORT_APP("morm_sp")
//#include "../../applications/morm_sp/morm_sp.h"
#include "../../applications/narwal-launcher/narwal-launcher.h"

//applications' entry-point (sort of)
void app_main(void)
{
	switch(hf_cpuid()){
		case 2:
		case 3:
		default:
			hf_spawn(receiver, 0, 0, 0, "narwal-launcher", 4096);			
			break;
	}
}


/*

void app_main(void)
{

	switch( hf_cpuid() ) {

	case 15: 
		hf_spawn(sender, 0, 0, 0, "sender", 4096);		
	        hf_spawn(receiver, 0, 0, 0, "receiver", 4096);
		break;

	case 5: hf_spawn(sender, 0, 0, 0, "sender", 4096);
		break;

	case 12: hf_spawn(sender, 0, 0, 0, "sender", 4096);	
		break;
	
	case RECV_ADDR: hf_spawn(receiver, 0, 0, 0, "receiver", 4096);
		break;	

	case 9: hf_spawn(receiver, 0, 0, 0, "receiver", 4096);
		break;

	default : break;

	}
	
	//if(hf_cpuid() == 2)
	//	hf_spawn(sender, 0, 0, 0, "sender", 4096);		
	//else if (hf_cpuid() == 3)
	//	hf_spawn(receiver, 0, 0, 0, "receiver", 4096);	
}
*/
