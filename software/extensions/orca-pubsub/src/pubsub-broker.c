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

#include "pubsub-shared.h" /* framework-related general purpose datatypes */
#include "pubsub-broker.h" /* broker-specific prototypes and definitions */



/**
 * @brief Broker task. Spawn this task wherever you 
 * need a broker. Please note that brokers are independent
 * from each other, so groups of communicant tasks must 
 * address the same broker(s). Task functions have no parameters.
 */
void pubsub_broker_tsk(void){

	int8_t buf[500];
	uint16_t cpu, port, size;
	int16_t val;
	
	pubsub_entry_t publishers[PUBSUBLIST_SIZE], subscribers[PUBSUBLIST_SIZE];
	pubsub_entry_t p, e;
	
	pubsublist_init(publishers);
	pubsublist_init(subscribers);	

	//we create brokers always using the same port, see <pubsub-shared.h>
	if (hf_comm_create(hf_selfid(), PS_BROKER_DEFAULT_PORT, 0))
		panic(0xff);
	
	PS_DEBUG("brk: started\n");
	
	//keep running indefinitely
	while (1){

		int32_t i = hf_recvprobe(); //check whether some message arrived at channel i

		if(i >= 0){

			val = hf_recv(&cpu, &port, buf, &size, i);

			if (val){
				printf("hf_recv(): error %d\n", val);
				
			} else {
				
				e = *((pubsub_entry_t*)buf);
				
				switch(e.opcode){

					/** handle a message for subscription
					 * a) add the subscriber to the table. if subscriber is already in the table, 
					 *    ignore.
					 * b) if subscriber is not in the table, notify publishers of that topic **/
					case PSMSG_SUBSCRIBE:{
					
						PS_DEBUG("brk: recv subscription\n");
						
						//try to insert in the list
						e.opcode = 0x1; // <== enable the entry
						val = pubsublist_add(subscribers, e);
						
						//if not in the list
						if(!val){
								
							PS_DEBUG("brk: registered subscription\n");
							PS_DEBUG("brk: cpu %d, topic %d, port %d\n", e.cpu, e.topic, e.port);
												
							//notify each publisher (clients)
							for(int i = 0; i < PUBSUBLIST_SIZE; i++){
								
								p = publishers[i];
								
								if(p.topic == e.topic){
									
									val = hf_send(p.cpu, PS_CLIENT_DEFAULT_PORT, (int8_t*)&e, sizeof(e), p.channel);
									
									if(val)
										printf("morreu");	
										
									PS_DEBUG("brk: notified publisher cpu %d, port %d, topic %d,\n", p.cpu, p.port, p.topic);
								}
							}	
						}
					} break;

					/** handle a message for unsubscription
					 * a) remove the subscriber from the table, if any
					 * b) if subscriber WAS in the table, notify publishers of that topic **/
					case PSMSG_UNSUBSCRIBE:{
						
						//try to remove from the list
						val = pubsublist_remove(subscribers, e);
						
						//if it WAS the list
						if(!val){
							
							//notify each publisher
							for(int i = 0; i < PUBSUBLIST_SIZE; i++){
								
								p = publishers[i];
								
								if(p.topic == e.topic){
									
									val = hf_send(p.cpu, p.port, (int8_t*)&e, sizeof(e), p.channel);
									
									if(val)
										printf("morreu");	
								}
							}
						}
					} break;

					/** handle a message for advertising
					 * a) add the advertiser to the table, if it is not there already
					 * b) if it wasn't in the table, send a list of subscribers **/
					case PSMSG_ADVERTISE:
						
						PS_DEBUG("brk: recv advertise\n");
						
						//operation is replaced by the "entry ok" flag
						e.opcode = 0x1; 
						
						//try to add to the list
						val = pubsublist_add(publishers, e);
						
						//if it was't in the list, send related subscribers
						if(!val){
							
							PS_DEBUG("brk: reg cpu %d, port %d, topic %d\n", 
								e.cpu, e.port, e.topic);
							
							//send the list of subscribers to the publisher
							for(int i = 0; i < PUBSUBLIST_SIZE; i++){
																
								p = subscribers[i];
															
								//check whether the entry has the same target topic AND is valid
								if(p.topic == e.topic && p.opcode == 0x1){
									
									PS_DEBUG("brk: ret sub cpu %d, port %d, topic %d\n", 
										p.cpu, p.port, p.topic);
									
									//must send to the client
									val = hf_send(e.cpu, PS_CLIENT_DEFAULT_PORT, (int8_t*)&p, sizeof(p), e.channel);
									
									if(val){
											//message has failed, ...
									}	
								}									
							}
							
							PS_DEBUG("brk: recv advertise done\n");
							
						}else{
							//tried to advertise but was in the list already, nothing to do
						}
						
						break;

					/**
					 * handle a message for unadvertise
					 * a) remove the entry from the table **/
					case PSMSG_UNADVERTISE:{
					
						//try to remove from the table 
						val = pubsublist_remove(publishers, e); 
						
					}break;

					/** maybe the message is unkown... **/
					default:{
						printf("could not handle pubsub message due to invalid opcode!\n");
					}

				}

				//printf("cpu %d, port %d, ch %d, size %d, #%d [free queue: %d]\n",
				//cpu, port, i, size, counter, hf_queue_count(pktdrv_queue));
				//counter++;
			}
		}
	}
}