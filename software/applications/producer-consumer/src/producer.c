#include <hellfire.h>
#include <noc.h>

#include "producer-consumer.h"


#define PRODUCE_LENGTH 32

void producer(void){

    int8_t buf[PRODUCE_LENGTH];
    int16_t val, channel, node;
    uint32_t counter;
	
    if (hf_comm_create(hf_selfid(), 1000, 0)){
        panic(0xff);
    }

    //delay necessary for the kernel to 
    //create the comm
   	delay_ms(2);
	
    //srand(hf_cpuid());
    
    node = 3;
    counter = 0;
    channel = hf_cpuid();
    
    while (1){

        //generate a bunch of random values
		//for (int i = 0; i < PRODUCE_LENGTH; i++)
			//buf[i] = random() % 255;
			//buf[i] = 0xAA;
		//	buf[i] = i;
		*((uint32_t*)buf) = counter;

		//send buffer data through the network. We use 
		//mod NUM_NODES to send it to a random node within
		//the system but for zero.
		//node = (random() % (hf_ncores() - 1)) + 1;

        //send first 32 bytes to some random node
        //val = hf_send(node, 5000, buf, PRODUCE_LENGTH, channel);
        val = hf_send(node, 5000, buf, sizeof(uint32_t), channel);

		if (val){
			printf("hf_send(): error %d\n", val);
		}else{
			printf("hf_send(): ch=%d, cpu=%d, size=%d, #%d\n", 
				//channel, node, PRODUCE_LENGTH, counter);
				channel, node, sizeof(uint32_t), *((uint32_t*)buf));
		}

		counter++;

		//add some delay to avoid flooding the network
		delay_ms(1);
	}
	
	//hf_kill(hf_selfid());
	//while(1);
}

