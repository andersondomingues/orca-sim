#include "noc_test4.h"

void noc_test4_receiver(void)
{
	
	int8_t buf[1500];
	uint16_t cpu, port, size;
	int16_t val;
	uint32_t crc;
	int32_t i;
	
	if (hf_comm_create(hf_selfid(), 5000, 0))
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
