#include <hellfire.h>
#include <noc.h>


void sender(void)
{
	int32_t i;
	uint32_t crc;
	int8_t buf[1000];
	int16_t val, channel;
	
	if (hf_comm_create(hf_selfid(), 1000, 0))
		panic(0xff);

	delay_ms(50);
	
	srand(hf_cpuid());
	
	// generate a unique channel number for this CPU
	channel = hf_cpuid();
	while (1){

		for (i = 0; i < sizeof(buf)-4; i++)
			buf[i] = random() % 255;
			
		crc = hf_crc32(buf, sizeof(buf)-4);
		memcpy(buf+sizeof(buf)-4, &crc, 4);
		val = hf_send(3, 5001, buf, sizeof(buf), channel);

		if (val)
			printf("hf_send(): error %d\n", val);
		else
			printf("hf_send(): channel=%d\n", channel);
			
		delay_ms(10);
		
		while(1);
	}
	

}

void receiver(void)
{
	int8_t buf[1500];
	uint16_t cpu, port, size;
	int16_t val;
	uint32_t crc;
	int32_t i;
	
	if (hf_comm_create(hf_selfid(), 5001, 0))
		panic(0xff);
	
	while (1){
		i = hf_recvprobe();
		if (i >= 0) {
			val = hf_recv(&cpu, &port, buf, &size, i);
			if (val){
				printf("hf_recv(): error %d\n", val);
			} else {
				memcpy(&crc, buf+size-4, 4);
								
				printf("cpu %d, port %d, channel %d, size %d, crc %08x [free queue: %d]", cpu, port, i, size, crc, hf_queue_count(pktdrv_queue));

				if (hf_crc32(buf, size-4) == crc)
					printf(" (CRC32 pass)\n");
				else
					printf(" (CRC32 fail)\n");
			
			}
		}
	}
}


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
			}else{
			
				printf("cpu=%d, task=%d, size=%d\n", cpu, task, size);
			}
				
			//alloc space for incoming application
			//hf_malloc(size);
			
			//copy application from buffer to memory
			//hf_memcopy();
			
			//spawn task
			//hf_spawn(sender, 0, 0, 0, "sender", 4096);
			
			//printf("recv'd: size=%d, cpu=%d, task=%d\n", size, cpu, task);
			
			//hexdump(buf, size);
			//printf("\n");
		}
	}
}

void app_main(void)
{
	hf_spawn(spawner, 0, 0, 0, "spawner", 4096);		
	
	if(hf_cpuid() == 2)
		hf_spawn(sender, 0, 0, 0, "sender", 4096);		
	else if (hf_cpuid() == 3)
		hf_spawn(receiver, 0, 0, 0, "receiver", 4096);	
}
