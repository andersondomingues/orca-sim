/*
 * Implementation file for ORCA-LIB library.
 * Copyright (C) 2018-2019 Anderson Domingues, <ti.andersondomingues@gmail.com>
 * This file is part of project URSA (http://https://github.com/andersondomingues/ursa).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. */
#include "hellfire.h"
#include "noc.h"

#include "pubsub-publisher.h"
#include "pubsub-shared.h"

//store the list of subscribers only for the topics that 
//have publishers in the current node. VOLATILE is required
//for sharedmem.
pubsub_entry_t _psclient_subscribers[PUBSUBLIST_SIZE];

volatile uint16_t _psclient_enabled = 0;

//constrols whether the client process has been started in this cpu
//zero = "hasn't started yet"
volatile uint16_t pubsub_has_a_client_started_already = 0;

/**
 * @brief A task to update info coming from brokers to publishers in a node */
void _psclient_tsk(){

	int8_t buf[500];
	uint16_t cpu, port, size;
	int16_t val;

	pubsub_entry_t e;

	//we create brokers always using the same port, see <pubsub-shared.h>
	if (hf_comm_create(hf_selfid(), PS_CLIENT_DEFAULT_PORT, 0))
		panic(0xff);

	//delay necessary for the kernel to create the comm
	delay_ms(60);
	
	//temporarily disable publishers
	_psclient_enabled = 0; 
	
	//reset list of subscribers
	pubsublist_init(_psclient_subscribers);

	//enable publishers again
	_psclient_enabled = 1; //enable publishers

	PS_DEBUG("[PS] client started\n");

	//keep running indefinitely
	while (1){

		int32_t i = hf_recvprobe(); //check whether some message arrived at channel i

		if(i >= 0){

			val = hf_recv(&cpu, &port, buf, &size, i);

			if (val){
				printf("hf_recv(): error %d\n", val);
				
			} else {
				
				e = *((pubsub_entry_t*)buf);
				
				//clients can handle only subscriptions and unsibscriptions
				switch(e.opcode){

					//add a subscriber to the table, if not there already 
					case PSMSG_SUBSCRIBE:{
						
						if(!pubsublist_has(_psclient_subscribers, e)){

							PS_DEBUG("[PS] add sub cpu %d, port %d, topic %d\n",
								e.cpu, e.port, e.topic);
						
							//try to insert in the list
							val = pubsublist_add(_psclient_subscribers, e);
						}
						
					} break;

					//remove a subscriber from the table, if not removed yet
					case PSMSG_UNSUBSCRIBE:{
						
						if(pubsublist_has(_psclient_subscribers, e)){

							//try to remove from the list
							val = pubsublist_remove(_psclient_subscribers, e);
							
							PS_DEBUG("[PS]: rem sub cpu %d, port %d, topic %d\n",
								e.cpu, e.port, e.topic);
						}
					} break;

					//invalid opcode? 
					default:{
						PS_DEBUG("GODAMN MAN YOU CRACK THE CAN OPEN AGAIN\n");
					}

				}
			}
		}
	}
}

/**
 * @brief Advertises to a topic
 * @param pubinfo Information to be stored in the table of publishers at the broker
 * @param brokerinfo Information about the broker (required since many brokers can run at once)
 * @param topic_name Name of the topic to which the publisher will publish
 */
void pubsub_advertise(pubsub_node_info_t pubinfo, pubsub_node_info_t brokerinfo, topic_t topic_name){
	
	//start a new client, if none has been started yet
	if(pubsub_has_a_client_started_already == 0){
		hf_spawn(_psclient_tsk, PS_CLIENT_PERIOD, PS_CLIENT_CAPACITY, PS_CLIENT_DEADLINE, "ps-client-task", PS_CLIENT_STACKSIZE);
		while(!_psclient_enabled);
	}
		
	//we always increment this counter to the number of current active "advertisers"
	//so that, when no advertisers is there, we can kill the client
	pubsub_has_a_client_started_already++;
		
	//create a new entry to be put in the publishers table and send to the broker
	pubsub_entry_t e = {
		.opcode = PSMSG_ADVERTISE,
		.cpu = pubinfo.address,
		.port = pubinfo.port,
		.topic = topic_name,
		.channel = 0  //TODO: is channel required?
	};
	
	PS_DEBUG("[PS] adv cpu %d, port %d, topic %d\n",
		e.opcode, e.cpu, e.port, e.topic);
	
	//TODO: is channel required?
	hf_send(brokerinfo.address, brokerinfo.port, (int8_t*)&e, sizeof(e), e.channel);
}

/**
 * @brief Unadvertise from a topic
 * @param broker_cpu_id CPU in which the broker is running in
 * @param topic Topic to which unsubscribe from
 */
void pubsub_unadvertise(pubsub_node_info_t pubinfo, pubsub_node_info_t brokerinfo, topic_t topic_name){
	
	//decrease the number of advertisers in one
	pubsub_has_a_client_started_already--;
	
	//TODO: if no advertisers is active, kill the client 
	//hf_kill(client)
	
	//notify the broker that the process is not a publish anymore
	pubsub_entry_t e = {
		.opcode = PSMSG_UNADVERTISE,
		.cpu = hf_cpuid(), //from noc.h
		.channel = 0,
		.port = pubinfo.port
	};
	
	PS_DEBUG("[PS] unadv cpu %d, port %d, topic %d\n",
		e.opcode, e.cpu, e.port, e.topic);
	
	hf_send(brokerinfo.address, brokerinfo.port, (int8_t*)&e, sizeof(e), e.channel);
}

/**
 * @brief Publish a message to some topic
 * @param topic Topic to which the message will be published
 * @param msg Message to be published
 * @param size Size of the message (in bytes) */
void pubsub_publish(topic_t topic, int8_t* msg, uint16_t size){
	
	//look for subscribers in the internal table
	pubsub_entry_t e;
	
	for(int i = 0; i < PUBSUBLIST_SIZE; i++){
		
		e = _psclient_subscribers[i];
		
		//skip invalid entries
		if(e.opcode != 0 && e.topic == topic){	
				
			//send the message to the subscriber
			hf_send(e.cpu, e.port, msg, size, e.channel);
			
			PS_DEBUG("[PS] pub cpu %d, port %d, topic %d\n",
				e.cpu, e.port, e.topic);
		}
	}
	
	// pubsublist_print(_psclient_subscribers);
}
