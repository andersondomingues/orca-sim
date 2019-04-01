#include <hellfire.h>
#include <noc.h>

#define MAX_TASK_SIZE 2500

uint8_t task_container[10000];

void spawner_listener(void)
{
	int8_t buf[MAX_TASK_SIZE];

	uint16_t cpu, task, size;
	int32_t val, i;

	if (hf_comm_create(hf_selfid(), 5000, 0))
		panic(0xff);

	//keep probing for new requests
	while (1){
	
		i = hf_recvprobe();

		//read received packet	
		if (i >= 0) {
	
			val = hf_recv(&cpu, &task, buf, &size, i);
	
			if (val){
				printf("hf_recv(): error %d\n", val);
			}else{
				printf("cpu=%d, task=%d, size=%d\n", cpu, task, size);
				
				void (*fun_ptr)(void) = (void (*)(void))buf;
				
				hexdump(buf, size);
				hf_spawn(fun_ptr, 0, 0, 0, "migrated_task_001", 4096);
			}
		}
	}
}

void app_main(void)
{
	hf_spawn(spawner_listener, 0, 0, 0, "spawner", 4096);		
	
	//if(hf_cpuid() == 2)
	//	hf_spawn(sender, 0, 0, 0, "sender", 4096);		
	//else if (hf_cpuid() == 3)
	//	hf_spawn(receiver, 0, 0, 0, "receiver", 4096);	
}
