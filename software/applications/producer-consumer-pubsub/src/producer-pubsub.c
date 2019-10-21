#include <hellfire.h>
#include <noc.h>

#include "producer-consumer-pubsub.h"
#include "../../orca-pubsub/include/pubsub-shared.h"
#include "../../orca-pubsub/include/pubsub-publisher.h"

#define PRODUCE_LENGTH 32

#define BROKER_ADDR 3
#define BROKER_PORT PS_BROKER_DEFAULT_PORT

#define PUBLISHER_PORT 2000

void producer_pubsub(void){
	
	int8_t buf[PRODUCE_LENGTH];

	// "opens" the comm
	if(hf_comm_create(hf_selfid(), PUBLISHER_PORT, 0))
		panic(0xff);

	//delay necessary for the kernel to create the comm
	delay_ms(60);
	
	//broker info (design time config.)
	pubsub_node_info_t pubinfo = {
		.address = BROKER_ADDR,
		.port    = BROKER_PORT
	};
	
	//this node
	pubsub_node_info_t brokerinfo = {
		.address = hf_selfid(),
		.port    = PUBLISHER_PORT //TODO: get this automatically?
	}; 

	//advertise to TOPIC_01, advertiser resides in port 2000
	pubsub_advertise(pubinfo, brokerinfo, TOPIC_01);

	//keep producing messages  
	while(1){

		//generate a bunch of values
		for (int i = 0; i < PRODUCE_LENGTH; i++)
			buf[i] = i;
		
		//publishes to the topic
		pubsub_publish(TOPIC_01, buf, sizeof(buf));
	}
}

