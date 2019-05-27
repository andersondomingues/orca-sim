#include <hellfire.h>
#include <noc.h>

#include "example-echo-print.h"

void example_echo_print(void)
{
    int8_t buf[500];
    uint16_t cpu, port, size;
    int16_t val;

    if (hf_comm_create(hf_selfid(), 5000, 0))
        panic(0xff);
	
    while (1){
	
        int32_t i = hf_recvprobe();
	
        if(i >= 0){

            val = hf_recv(&cpu, &port, buf, &size, i);
	
            if (val){
		printf("hf_recv(): error %d\n", val);
	    } else {		
	        printf("cpu %d, port %d, channel %d, size %d, [free queue: %d]\n",
                        cpu, port, i, size, hf_queue_count(pktdrv_queue));
                printf("content [\n");
    
                for(int j = 0; j < size; j++)
                    printf("%c", buf[j]);
                printf("]\n");
	    }
	}
    }
}
