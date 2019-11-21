#include <hellfire.h>
#include <noc.h>

#include "producer-consumer.h"

void consumer(void)
{
    int8_t buf[500];
    uint16_t cpu, port, size;
    uint32_t counter;
    int16_t val;

    if (hf_comm_create(hf_selfid(), 5000, 0)){
        panic(0xff);
    }

	delay_ms(2);
    counter = 0;
	
	//printf("buf: 0x%x\n", (uint32_t)buf);
	
	//while(1);
	
    while (1){
	
        int32_t i = hf_recvprobe();
	
        if(i >= 0){

            val = hf_recv(&cpu, &port, buf, &size, i);
	
            if (val){
				printf("hf_recv(): error %d\n", val);
            } else {
	        	//printf("#%d [free queue: %d]\n", counter, hf_queue_count(pktdrv_queue));
	        	//printf("cpu %d, port %d, ch %d, size %d\n", cpu, port, i, size);
	        	printf("content %d\n", *((uint32_t*)buf));

				//if(counter != *((uint32_t*)buf))
				//	hf_kill(hf_selfid());
				//	printf("oomp!\n");

				//printf("\n------\n");
	        	//hexdump(buf, sizeof(uint32_t) * 2);
				//printf("\n");
				
                counter++;
	    	}
		}
    }
}
