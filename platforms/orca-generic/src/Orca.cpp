/** 
 * Implementation file for ORCA-SIM program.
 * This file is part of project URSA. http://https://github.com/andersondomingues/ursa
 *
 * Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. **/
 
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

//signal manip
#include <signal.h>

//simulation artifacts (from URSA)
#include <Event.h>
#include <Simulator.h>

//built-in models (from URSA)
#include <UMemory.h>

//reusable models
#include <THellfireProcessor.h>
#include <TRouter.h>
#include <TNetif.h> 
#include <TNetSocket.h>

//orca-specific hardware
#include <Tile.h>

#include <ProcessingTile.h>
#include <NetworkTile.h>

//instantiates a mesh of MxN PE
Tile* tiles[ORCA_NOC_WIDTH][ORCA_NOC_HEIGHT];

//interrupt signal catcher
static volatile sig_atomic_t interruption = 0;

static void sig_handler(int _){
	
	(void)_;
	
	switch(interruption){
	case 0:
		interruption = 1;
		std::cout << std::endl << "Simulation interrupted. Wait for the current epoch to finish or press CTRL+C again to force quit." << std::endl; 		
		break;
	case 1:
		exit(0);
		break;
	case 2:
		std::cout << std::endl << "Hold your horses!" << std::endl;
		break;
	}	
}

//connect routers to each other
void connect_routers(TRouter* r1, uint32_t p1, TRouter* r2, uint32_t p2){
	r1->SetOutputBuffer(r2->GetInputBuffer(p2), p1);
	r2->SetOutputBuffer(r1->GetInputBuffer(p1), p2);
	
	std::cout << "router_comm: " << r1->GetName() << " ----[" 
			  << p1 << "/" << p2 << "]---- " << r2->GetName() << std::endl;
}

void check_params(){

	//orca params 
	#ifndef ORCA_NOC_HEIGHT
	std::runtime_error("ORCA_NOC_HEIGHT must be defined in Configuration.mk\n");
	#else
	std::cout << "ORCA_NOC_HEIGHT set to " << ORCA_NOC_HEIGHT << std::endl;
	#endif
	
	#ifndef ORCA_NOC_HEIGHT
	std::runtime_error("ORCA_NOC_WIDTH must be defined in Configuration.mk\n");
	#else
	std::cout << "ORCA_NOC_WIDTH set to " << ORCA_NOC_HEIGHT << std::endl;
	#endif

	#ifndef ORCA_EPOCH_LENGTH
	std::runtime_error("ORCA_EPOCH_LENGTH must be defined in Configuration.mk\n");
	#else
	std::cout << "ORCA_EPOCH_LENGTH set to " << ORCA_EPOCH_LENGTH << std::endl;
	#endif

	#ifndef ORCA_EPOCHS_TO_SIM
	std::cout << "ORCA_EPOCHS_TO_SIM set to INFINITE" << std::endl;
	#else
	std::cout << "ORCA_EPOCHS_TO_SIM set to " << ORCA_EPOCHS_TO_SIM << std::endl;
	#endif
	
	//ursa params
	#ifndef URSA_ZERO_TIME_CHECKING
	std::cout << "URSA_ZERO_TIME_CHECKING disabled" << std::endl;
	#else
	std::cout << "URSA_ZERO_TIME_CHECKING set to " << URSA_ZERO_TIME_CHECKING << std::endl;
	#endif

	#ifndef URSA_QUEUE_SIZE_CHECKING
	std::cout << "URSA_QUEUE_SIZE_CHECKING disabled" << std::endl;
	#else
	std::cout << "URSA_QUEUE_SIZE_CHECKING set to " << URSA_QUEUE_SIZE_CHECKING << std::endl;
	#endif
	
	//netsocket logs
	#ifndef NETSOCKET_LOG_OUTGOING_PACKETS
	std::cout << "NETSOCKET_LOG_OUTGOING_PACKETS disabled" << std::endl;
	#else
	std::cout << "NETSOCKET_LOG_OUTGOING_PACKETS enabled" << std::endl;
	#endif

	#ifndef NETSOCKET_LOG_INCOMING_PACKETS
	std::cout << "NETSOCKET_LOG_INCOMING_PACKETS disabled" << std::endl;
	#else
	std::cout << "NETSOCKET_LOG_INCOMING_PACKETS enabled" << std::endl;
	#endif
	
	#ifdef NETSOCKET_CLIENT_ADDRESS
	std::cout << "NETSOCKET_CLIENT_ADDRESS is " << NETSOCKET_CLIENT_ADDRESS << std::endl;
	#endif
	
	#ifdef NETSOCKET_CLIENT_PORT
	std::cout << "NETSOCKET_CLIENT_PORT is " << NETSOCKET_CLIENT_PORT << std::endl;
	#endif
	
	#ifdef NETSOCKET_SERVER_ADDRESS
	std::cout << "NETSOCKET_SERVER_ADDRESS is " << NETSOCKET_SERVER_ADDRESS << std::endl;
	#endif
	
	#ifdef NETSOCKET_SERVER_PORT
	std::cout << "NETSOCKET_SERVER_PORT is " << NETSOCKET_SERVER_PORT << std::endl;
	#endif
	
	//buffers
	#ifndef BUFFER_OVERFLOW_CHECKING
	std::cout << "BUFFER_OVERFLOW_CHECKING disabled" << std::endl;
	#else
	std::cout << "BUFFER_OVERFLOW_CHECKING enabled" << std::endl;
	#endif

	#ifndef BUFFER_UNDERFLOW_CHECKING
	std::cout << "BUFFER_UNDERFLOW_CHECKING disabled" << std::endl;
	#else
	std::cout << "BUFFER_UNDERFLOW_CHECKING enabled" << std::endl;
	#endif
	
	//memory
	#ifndef MEMORY_WRITE_ADDRESS_CHECKING
	std::cout << "MEMORY_WRITE_ADDRESS_CHECKING disabled" << std::endl;
	#else
	std::cout << "MEMORY_WRITE_ADDRESS_CHECKING enabled" << std::endl;
	#endif

	#ifndef MEMORY_READ_ADDRESS_CHECKING
	std::cout << "MEMORY_READ_ADDRESS_CHECKING disabled" << std::endl;
	#else
	std::cout << "MEMORY_READ_ADDRESS_CHECKING enabled" << std::endl;
	#endif

	#ifndef MEMORY_WIPE_ADDRESS_CHECKING
	std::cout << "MEMORY_WIPE_ADDRESS_CHECKING disabled" << std::endl;
	#else
	std::cout << "MEMORY_WIPE_ADDRESS_CHECKING enabled" << std::endl;
	#endif

	#ifndef MEMORY_ENABLE_COUNTERS
	std::cout << "MEMORY_ENABLE_COUNTERS disabled" << std::endl;
	#else
	std::cout << "MEMORY_ENABLE_COUNTERS enabled" << std::endl;
	#endif
	
	//hfriscv
	#ifndef HFRISCV_WRITE_ADDRESS_CHECKING
	std::cout << "HFRISCV_WRITE_ADDRESS_CHECKING disabled" << std::endl;
	#else
	std::cout << "HFRISCV_WRITE_ADDRESS_CHECKING enabled" << std::endl;
	#endif

	#ifndef HFRISCV_READ_ADDRESS_CHECKING
	std::cout << "HFRISCV_READ_ADDRESS_CHECKING disabled" << std::endl;
	#else
	std::cout << "HFRISCV_READ_ADDRESS_CHECKING enabled" << std::endl;
	#endif

	#ifndef HFRISCV_ENABLE_COUNTERS
	std::cout << "HFRISCV_ENABLE_COUNTERS disabled" << std::endl;
	#else
	std::cout << "HFRISCV_ENABLE_COUNTERS enabled" << std::endl;
	#endif
	
	//router	
	#ifndef ROUTER_ENABLE_COUNTERS
	std::cout << "ROUTER_ENABLE_COUNTERS disabled" << std::endl;
	#else
	std::cout << "ROUTER_ENABLE_COUNTERS enabled" << std::endl;
	#endif
	
	#ifndef ROUTER_PORT_CONNECTED_CHECKING
	std::cout << "ROUTER_PORT_CONNECTED_CHECKING disabled" << std::endl;
	#else
	std::cout << "ROUTER_PORT_CONNECTED_CHECKING enabled" << std::endl;
	#endif
	
}

int main(int __attribute__((unused)) argc, char** argv){

    //argc = argc; //workaround to use -Wextra

	//register interruption handler
	signal(SIGINT, sig_handler);

	std::cout << "URSA/ORCA Platform " << std::endl;

	std::cout << "==============[ PARAMETERS ]" << std::endl;	
	try{
		check_params();
	}catch(std::runtime_error& e){
		std::cout << e.what() << std::endl;
		return 1;
	}
	
	std::cout << "==============[ TILE COMPOSITION ]" << std::endl;
	
	//populate tiles
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
			
			if(x == 0 && y ==0){
				std::cout << "[N]";
				tiles[x][y] = (Tile*)new NetworkTile(x, y);				
			}else{
				std::cout << "[P]";
				tiles[x][y] = (Tile*)new ProcessingTile(x, y);
			}
		}
		
		std::cout << std::endl;
	}
	
	std::cout << "==============[ ROUTER CONNECTIONS ]" << std::endl;
	
	//connect tiles to each other (left-to-right, right-to-left connections)	
	for(int x = 0; x < ORCA_NOC_WIDTH - 1; x++)
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++)
			connect_routers(tiles[x][y]->GetRouter(), EAST, tiles[x+1][y]->GetRouter(), WEST);
			
	//connect tiles to each other (bottom-to-top, top-to-bottom connections)
	for(int x = 0; x < ORCA_NOC_WIDTH; x++)
		for(int y = 0; y < ORCA_NOC_HEIGHT - 1; y++)
			connect_routers(tiles[x][y]->GetRouter(), NORTH, tiles[x][y+1]->GetRouter(), SOUTH);
			
	//load binaries into main memories (processing tiles only)
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
			
			//zero-zero is for network interface
			if(x == 0 && y == 0) continue;
		
			//index = x + ORCA_NOC_WIDTH * y;
			((ProcessingTile*)tiles[x][y])->GetMem0()->LoadBin(std::string(argv[1]), MEM0_BASE, MEM0_SIZE);
		}
	}

	std::cout << "==============[ SIMULATION ]" << std::endl;
	
	//instantiate simulation
	Simulator* s = new Simulator();
		
	std::cout << "Scheduling..."	 << std::endl;
	
	//schedule hardware to be simulated
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
			
			//netork tile
			if(x == 0 && y == 0)
				s->Schedule(Event(1, ((NetworkTile*)tiles[x][y])->GetSocket()));
				
			//processing tile
			else
				s->Schedule(Event(4, ((ProcessingTile*)tiles[x][y])->GetCpu()));
			
			s->Schedule(Event(1, tiles[x][y]->GetRouter()));
			s->Schedule(Event(1, tiles[x][y]->GetNetif()));
		}
	}

	std::cout << "Epoch set to " << ORCA_EPOCH_LENGTH << " cycles." << std::endl;
	std::cout << "Please wait..." << std::endl;

	try{
	
		std::chrono::high_resolution_clock::time_point t1, t2;
	
		while(!interruption){
			
			t1 = std::chrono::high_resolution_clock::now();
			s->Run(ORCA_EPOCH_LENGTH);
			t2 = std::chrono::high_resolution_clock::now();
			
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
			s->NextEpoch();
			
			//converts mili to seconds before calculating the frequency
			double hertz = ((double)ORCA_EPOCH_LENGTH) / ((double)((double)duration / 1000.0));

			//divide frequency by 1k (Hz -> KHz)
			std::cout << "notice: epoch #" << s->GetEpochs() << " took ~" 
                                << duration << "ms (running @ " << (hertz / 1000000.0)
				<< " MHz)" << std::endl;
							
			//simulate until reach the limit of pulses
			#ifdef ORCA_EPOCHS_TO_SIM
			if(s->GetEpochs() >= ORCA_EPOCHS_TO_SIM)
				break;
			#endif
		}
		
	}catch(std::runtime_error& e){
		std::cout << e.what() << std::endl;
		goto clean;
	}
	
	std::cout << "Simulation ended without errors."	 << std::endl;
	
	//show buffer status
	std::cout << "==============[ BUFFERS' STATUSES ]" << std::endl;
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
		
			TRouter* r = tiles[x][y]->GetRouter();
			std::cout << r->GetName() << ":"
				<< " S=" << r->GetInputBuffer(SOUTH)->size()
				<< " N=" << r->GetInputBuffer(NORTH)->size() 
				<< " W=" << r->GetInputBuffer(WEST)->size()
				<< " E=" << r->GetInputBuffer(EAST)->size()
				<< " L=" << r->GetInputBuffer(LOCAL)->size() 
				<< " RR=" << r->GetRR()
				<< std::endl;
		}
	}
	
	//NI states
	std::cout << "==============[ NIs STATUSES ]" << std::endl;
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
		
			TNetif* n = tiles[x][y]->GetNetif();
			std::cout << n->GetName() << ":"
				<< " Send=" << static_cast<unsigned int>(n->GetSendState())
				<< " Recv=" << static_cast<unsigned int>(n->GetRecvState()) << " |"
				<< " A:" << std::hex << (int)(n->GetCommAck()->Read())
				<< " I:" << std::hex << (int)(n->GetCommIntr()->Read())
				<< " S:" << std::hex << (int)(n->GetCommStart()->Read())
				<< " T:" << std::hex << (int)(n->GetCommStart()->Read())
				<< std::endl;
		}
	}
		
	delete(s);
	return 0;
	
clean:
	
	delete(s); //sim
	
	//delete PE
	for(int x = 0; x < ORCA_NOC_WIDTH; x++)
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++)
			delete(tiles[x][y]);

	return 1;	
}




