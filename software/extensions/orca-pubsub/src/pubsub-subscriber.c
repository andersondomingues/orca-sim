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
#ifndef __PUBSUB_CLIENT_H
#define __PUBSUB_CLIENT_H

#include "hellfire.h"
#include "noc.h"

#include "pubsub-shared.h"
#include "pubsub-subscriber.h"

/**
 * @brief Subscribes to a topic;
 * @param subinfo Network information of the subscriber
 * @param brokerinfo Network information of the broker
 * @param Topic to which to subscribe to
 */
void pubsub_subscribe(pubsub_node_info_t subinfo, pubsub_node_info_t brokerinfo, topic_t topic){
	
	//create a new pubsub entry
	pubsub_entry_t t = {
		.opcode = PSMSG_SUBSCRIBE,
		.topic = topic,
		.cpu  = subinfo.address,
		.port = subinfo.port
	};
	
	//PS_DEBUG("sub: msg opcode %d, topic %d, cpu %d, port %d\n", t.opcode, t.topic, t.cpu, t.port);
	//PS_DEBUG("sub: broker at cpu %d, port %d\n", brokerinfo.address, brokerinfo.port);
		
	//send entry to the broker
	hf_send(brokerinfo.address, brokerinfo.port, (int8_t*)(&t), sizeof(t), 0);
}

/**
 * @brief Unsubscribes from a topic;
 * @param subinfo Network information of the subscriber
 * @param brokerinfo Network information of the broker
 * @param Topic to unsubscribe from
 */
void pubsub_unsubscribe(pubsub_node_info_t subinfo, pubsub_node_info_t brokerinfo, topic_t topic){
	
	//create a new pubsub entry
	pubsub_entry_t t = {
		.opcode = PSMSG_UNSUBSCRIBE,
		.topic = topic,
		.cpu  = subinfo.address,
		.port = subinfo.port
	};
	
	//PS_DEBUG("sub: msg opcode %d, topic %d, cpu %d, port %d\n", t.opcode, t.topic, t.cpu, t.port);
	//PS_DEBUG("sub: broker at cpu %d, port %d\n", brokerinfo.address, brokerinfo.port);
		
	//send entry to the broker
	hf_send(brokerinfo.address, brokerinfo.port, (int8_t*)(&t), sizeof(t), 0);
}


#endif /* __PUBSUB_CLIENT_H */



