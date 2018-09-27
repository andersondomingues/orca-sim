/*-------------------------------------------------------------------------
# This file is part of Project URSA. More information on the project
# can be found at URSA's repository at GitHub.
# 
# http://https://github.com/andersondomingues/ursa
# 
# Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#-------------------------------------------------------------------------*/

#include <Event.h>
#include <Simulator.h>
#include <iostream>

#define CYCLES_TO_SIM 10000
#define PACKET_LEN 6

void test(int port, TRouter* router){

	//16 packets of PACKET_LEN size 
	//Don't let your eyes fool you! It looks like 20 but behaves like 16...
	FlitType packets[20][PACKET_LEN];
	
	//populate packets with random values
	for(int i = 0; i < 16; i++)
		for(int j = 0; j < PACKET_LEN; j++);
			packets[i][j] = rand();
	
	//populate 
	for(int i = 0; i < 5; i++){
		packets[i*4 + 0][0] = 
		packets[i*4 + 1] =
		packets[i*4 + 2] =
		packets[i*4 + 3] = i;
	}
		
	//ptr to simulation
	Simulator* s = new Simulator();
	

		std::cout << "Simulating for port " << i << std::endl;

		//reset router	
		r->Reset();
		
		//reset buffers
		for(int l = 0; l < 5; l++)
			b[l]->Reset();

		//simulate for all packages
		for(int j = 0; j < 6; j++){
		
			std::cout << "   packet sent from port " << i << ": ";
		
			//add packet flits to port
			for(int k = 0; k < 6; k++){
				r->GetInputBuffer(i)->push(packet[j][k]);
				std::cout << std::hex << packet[j][k] << " ";	
			}
		
			std::cout << std::endl;
		
			//simulate
			for(int cycles = 0; cycles < CYCLES_TO_SIM; cycles++){
				s->Run(1);
			}
			
			//check for expected results
			FlitType addr = packet[j][0];
			
			uint8_t tx = (addr & 0xFF00) >> 8;
			uint8_t ty = (addr & 0x00FF);
						
			std::cout << "      from (" << std::hex << (unsigned int)x << "," << std::hex 
				<< (unsigned int)y << ") to ("	<< std::hex << (unsigned int)tx << "," 
				<< std::hex << (unsigned int)ty << ")" << std::endl;
			
			int target_port = -1;

			//if X=0, then route "vertically" (Y)
			if(x == tx){
		
				target_port = (y == ty)
				    ? LOCAL
				    : (y > ty)
				        ? SOUTH
				        : NORTH;      
			//else route X
			}else{

				target_port = (x > tx)
				    ? WEST
				    : EAST;
			}
			
			std::cout << "      target port = " << target_port << std::endl;
			
			std::cout <<  (
				(b[target_port]->size() > 0) 
					? "      -> ok " 
					: "      -> nok ") << endl;
		}
	}
}