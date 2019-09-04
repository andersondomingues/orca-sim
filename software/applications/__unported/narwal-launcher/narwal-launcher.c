#include <hellfire.h>
#include <noc.h>

#define RECV_ADDR 7


void sender(void)
{
	int32_t i;
	uint32_t crc;
	int8_t buf[1000];
	int16_t val, val2, val3, channel;
	
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

		val = hf_send(RECV_ADDR, 5001, buf, sizeof(buf), channel);
		val2 = hf_send(15, 5001, buf, sizeof(buf), channel);
		val3 = hf_send(9, 5001, buf, sizeof(buf), channel);

		if (val)
			printf("hf_send(): error %d\n", val);
		else
			printf("hf_send(): channel=%d\n", channel);

		if (val2)
			printf("hf_send(): error %d\n", val);
		else
			printf("hf_send(): channel=%d\n", channel);

		if (val3)
			printf("hf_send(): error %d\n", val);
		else
			printf("hf_send(): channel=%d\n", channel);
			
		delay_ms(10);
		
		//while(1);
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


