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
//have publishers in the current node
pubsub_entry_t _psclient_subscribers[PUBSUBLIST_SIZE];

/**
 * @brief Advertises to a topic;
 * @param broker_cpu_id CPU in which the broker is running
 * @param topic Topic to which the publisher is advertising */
void pubsub_advertise(uint16_t broker_cpu_id, topic_t topic){
	
	pubsub_entry_t e;
	e.opcode = PSMSG_UNADVERTISE;
	e.cpu = hf_cpuid(); //from noc.h
	e.channel = PS_BROKER_CHANNEL;
	e.port = 0x0; //dummy
	
	hf_send(broker_cpu_id, PS_BROKER_PORT, (int8_t*)&e, sizeof(e), PS_BROKER_CHANNEL);
}

/**
 * @brief Unadvertise from a topic
 * @param broker_cpu_id CPU in which the broker is running in
 * @param topic Topic to which unsubscribe from
 */
void pubsub_unadvertise(uint16_t broker_cpu_id, topic_t topic){
	
	pubsub_entry_t e;
	e.opcode = PSMSG_UNADVERTISE;
	e.cpu = hf_cpuid(); //from noc.h
	e.channel = PS_BROKER_CHANNEL;
	e.port = 0x0; //dummy
	
	hf_send(broker_cpu_id, PS_BROKER_PORT, (int8_t*)&e, sizeof(e), PS_BROKER_CHANNEL);
}

/**
 * @brief Publish a message to some topic
 * @param topic Topic to which the message will be published
 * @param msg Message to be published
 * @param size Size of the message (in bytes) */
void pubsub_publish(topic_t topic, int8_t* msg, uint16_t size){
	
	pubsub_entry_t e;
	
	for(int i = 0; i < PUBSUBLIST_SIZE; i++){
		
		e = _psclient_subscribers[i];
		
		//skip invalid entries
		if(e.opcode == 0)
			continue;
			
		//check whether the topic is the one to which the publisher is publishing
		if(e.topic != topic)
			continue;
			
		//send the message to the subscriber
		hf_send(e.cpu, e.port, msg, size, e.channel);
	}
}

