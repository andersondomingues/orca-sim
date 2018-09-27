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

#include <cstdlib>
#include <TRouter.h>

typedef uint16_t FlitType;

#define NUM_FLITS 6
#define FLIT_LEN  6

void test(int port, TRouter* router);

int main(int argc, char** argv){

	//initialize randomizer
	srand(time(NULL));

	uint8_t x = rand();
	uint8_t y = rand();

	//creates a new router at arbitrary position
	TRouter* r = new TRouter("test_router", x, y);

	//instantiate some output buffers to collect routed flits
	UBuffer<FlitType>* b[5] = {
		new UBuffer<FlitType>("o_north"), // 0 = NORTH
		new UBuffer<FlitType>("o_north"), // 1 = WEST
		new UBuffer<FlitType>("o_west"),  // 2 = SOUTH
		new UBuffer<FlitType>("o_east"),  // 3 = EAST
		new UBuffer<FlitType>("o_local")  // 4 = LOCAL
	};
	
	//attach buffers to the router
	for(int i = 0; i < 5; i++)
		r->SetOutputBuffer(b[i], i);
		
	//simulate for all ports 
	for(int i = 0; i < 5; i++)
		test(i, r);
}

#include "tests.cpp"