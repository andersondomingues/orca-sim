#include <hellfire.h>
#include <noc.h>

void spawner(void)
{

	int8_t buf[2500];
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
				continue;
			}
				
			//alloc space for incoming application
			//hf_malloc(size);
			
			//copy application from buffer to memory
			//hf_memcopy();
			
			//spawn task
			//hf_spawn(sender, 0, 0, 0, "sender", 4096);
			
			printf("msg recv'd with size=%d (0x%x)\n", size, size);
			
			hexdump(buf, size);
			printf("\n");
		}
	}
}

void app_main(void)
{
	hf_spawn(spawner, 0, 0, 0, "spawner", 4096);		
}
