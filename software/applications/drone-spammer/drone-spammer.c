/*
 * Implementation file for drone-spammer task.
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
 
#include "drone-spammer.h"

#include <hellfire.h>
#include <noc.h>

void dronespammer(void){

	//start new communication on port 5000
	if (hf_comm_create(hf_selfid(), 5000, 0))
		panic(0xff);
	
	//random seed based on cpuid
	srand(hf_cpuid());
			
	//send fake data periodically
	while(1){

		drone_ekf_data_t data;

		data.a = random();
		data.b = random(); 
		data.c = random();
		data.d = random(); 

		data.e = random();
		data.f = random(); 
		data.g = random();
		data.h = random(); 
		
		data.i = random();
		data.j = random(); 
		data.k = random();
		data.l = random();
		
		printf("drone-spammer: %d\n", data.a);
		
		//ekf processes resides in node 3, port 5000
		hf_send(3, 5000, (int8_t*)&data, sizeof(drone_ekf_data_t), 100);

		//wait for some time before next send
		delay_ms(10);
    }
}
