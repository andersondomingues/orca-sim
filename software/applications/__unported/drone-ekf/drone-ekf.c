/*
 * Implementation file for drone-ekf task.
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

#include "drone-ekf.h"

#include <hellfire.h>
#include <noc.h>

void droneekf(void){

   	int8_t buf[500];
	uint16_t cpu, port, size;
	int16_t val;
	
	//start new communication on port 5000
	if (hf_comm_create(hf_selfid(), 5000, 0))
		panic(0xff);

	while(1){
		
		//check whether a new packet has arrived
		int32_t i = hf_recvprobe();

		if(i >= 0){
		
			//recv packet from the buffer
			val = hf_recv(&cpu, &port, buf, &size, i);

			if (val){
				printf("hf_recv(): error %d\n", val);
				
			}else{
				
				//print packet info
				//printf("cpu %d, port %d, channel %d, size %d, [free queue: %d]\n",
				//	cpu, port, i, size, hf_queue_count(pktdrv_queue));

				//get received data					
				drone_ekf_data_t data;
				data = *(drone_ekf_data_t*)buf; //get data from buffer
			
				//make new data
				drone_pid_data_t datap;
				datap.a = data.a + data.b + data.c + data.d;
				datap.b = data.e + data.f + data.g + data.h;
				datap.c = data.i + data.j + data.k;
				datap.d = data.l;
				
				printf("drone-efk: %d\n", datap.d);
				
				//pid processes resides in node 2, port is 5000
				val = hf_send(2, 2000, (int8_t*)&datap, sizeof(drone_pid_data_t), 100);
			}
		}
	}
}
