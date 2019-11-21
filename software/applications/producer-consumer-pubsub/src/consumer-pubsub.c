#include <hellfire.h>
#include <noc.h>

#include "producer-consumer-pubsub.h"
#include "../../orca-pubsub/include/pubsub-shared.h"
#include "../../orca-pubsub/include/pubsub-subscriber.h"

#define BROKER_ADDR 3

#define SUBPORT 2006

void consumer_pubsub(void)
{
	int8_t buf[500];
	uint16_t cpu, port, size, counter = 0;
	int16_t val;

	if (hf_comm_create(hf_selfid(), SUBPORT, 0)){
		panic(0xff);
	}

	delay_ms(5);

	//info for this node (design time)
	pubsub_node_info_t subinfo = {
		.address = hf_cpuid(),
		.port = SUBPORT //TODO get this automatically
	};
	
	//broker info (design time)
	pubsub_node_info_t brokerinfo = {
		.address = BROKER_ADDR, //application-specific
		.port    = PS_BROKER_DEFAULT_PORT
	};

	//subscribe to some topic
	pubsub_subscribe(subinfo, brokerinfo, TOPIC_01);
	
	//receiving process proceeds as for ordinary messages
	while (1){
	
		//printf("counter: %d\n", counter);
		
		int32_t i = hf_recvprobe();
	
		if(i >= 0){

			val = hf_recv(&cpu, &port, buf, &size, i);

			if (val){
				printf("hf_recv(): error %d\n", val);
			} else {
				printf("cpu %d, port %d, ch %d, size %d, #%d [free queue: %d]\n",
					cpu, port, i, size, counter, hf_queue_count(pktdrv_queue));
			}
		}
		
		counter++;
	}

	//unsubscribe from TOPIC_01
	pubsub_unsubscribe(subinfo, brokerinfo, TOPIC_01);

	//terminate the application
	//hf_kill(hf_selfid());
	while(1);
}
