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
#include <THFRiscV.h>
#include <TRouter.h>
#include <TDmaNetif.h> 
#include <TNetBridge.h>

//orca-specific hardware
#include <Tile.h>

#include <ProcessingTile.h>
#include <NetworkTile.h>

//instantiates a mesh of MxN PE
Tile* tiles[ORCA_NOC_WIDTH][ORCA_NOC_HEIGHT];

//interrupt signal catcher
static volatile sig_atomic_t interruption = 0;

int _status = 0;

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
void connect_routers(HermesRouter* r1, uint32_t p1, HermesRouter* r2, uint32_t p2){
	
	r1->SetOutputBuffer(r2->GetInputBuffer(p2), p1);
	r2->SetOutputBuffer(r1->GetInputBuffer(p1), p2);
	
	//std::cout << "router_signal: " << r1->GetName() << " ----[" 
	//		  << p1 << "/" << p2 << "]---- " << r2->GetName() << std::endl;

	std::cout << r1->GetInputBuffer(p1)->GetName()
			  << " <---> " << r2->GetInputBuffer(p2)->GetName() << std::endl;
}

int main(int __attribute__((unused)) argc, char** argv){

    //display usage message
    if(argc < 2){
        std::cout << "usage: \n\t" << argv[0] << " <software-image>" << std::endl;
        return 1;
    }
    
    std::string param1 = std::string(argv[1]);

	//register interruption handler
	signal(SIGINT, sig_handler);

	std::cout << "URSA/ORCA Platform " << std::endl;
	std::cout << "==============[ TILE IDENTIFICATION ]" << std::endl;
	
	//populate tiles
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
			if(x == 0 && y == 0)
				std::cout << "[C]";
			else
				std::cout << "[P]";
		}
		std::cout << std::endl;
	}

	#ifdef ORCA_ENABLE_GDBRSP
	std::cout << "==============[ GDBRSP SERVERS ]" << std::endl;
	#endif
	

	//populate tiles
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
			
			if(x == 0 && y ==0){
				tiles[x][y] = (Tile*)new NetworkTile(x, y);
				std::cout << "[----]";				
			}else{
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
			((ProcessingTile*)tiles[x][y])->GetMem0()->LoadBin(param1, MEM0_BASE, MEM0_SIZE);
		}
	}

	std::cout << "==============[ SIMULATION ]" << std::endl;
	
	//instantiate simulation
	Simulator* s = new Simulator();
		
	std::cout << "Scheduling..."	 << std::endl;
	
	//schedule hardware to be simulated
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
			
			//schedule network bridge module, which starts at the first cycle
			if(x == 0 && y == 0){
				s->Schedule(Event(1, ((NetworkTile*)tiles[x][y])->GetSocket()));
				
			//schedule the cpu to start at the third cycle, because no instruction
			//gets out the cpu before that cycle
			}else{
				s->Schedule(Event(3, ((ProcessingTile*)tiles[x][y])->GetCpu()));
				s->Schedule(Event(1, ((ProcessingTile*)tiles[x][y])->GetDmaNetif()));
			}
			
			//all other hardware start at the first cycle
			s->Schedule(Event(1, tiles[x][y]->GetRouter()));

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

			#ifdef ORCA_EPOCHS_TO_SIM
			//simulate until reach the limit of pulses
			if(s->GetEpochs() >= ORCA_EPOCHS_TO_SIM)
				break;
			#endif
		}
		
	}catch(std::runtime_error& e){
		std::cout << e.what() << std::endl;
		_status = 1;
	}
	
	//show buffer status
	std::cout << "==============[ BUFFERS' STATUSES ]" << std::endl;
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
		
			HermesRouter* r = tiles[x][y]->GetRouter();
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
		
			if(x == 0 && y == 0) continue;
		
			DmaNetif* n = ((ProcessingTile*)(tiles[x][y]))->GetDmaNetif();
			std::cout << n->GetName() << ":"
				<< " SEND_STATE=" << static_cast<unsigned int>(n->GetSendState())
				<< " RECV_STATE=" << static_cast<unsigned int>(n->GetRecvState())
				// << " |"
				<< std::endl;
		}
	}

	//CPU statuses
	std::cout << "==============[ CPU STATUSES ]" << std::endl;
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
		
			if(x == 0 && y == 0)
				continue;
			
			ProcessingTile* t = (ProcessingTile*)(tiles[x][y]);
			HFRiscV* n = t->GetCpu();
			std::cout 
				<< n->GetName() 
				<< ": INTR=" << (int)(n->GetSignalIntr()->Read()) 
				<< ", STALL=" << (int)(n->GetSignalStall()->Read()) << std::endl;
				
			((ProcessingTile*)tiles[x][y])->GetMem0()->
				SaveBin(std::string(argv[1]) + n->GetName() + ".save.bin", MEM0_BASE, MEM0_SIZE);
		}
	}

	delete(s);
		
	//delete PE
	for(int x = 0; x < ORCA_NOC_WIDTH; x++){
		for(int y = 0; y < ORCA_NOC_HEIGHT; y++){
			
			if(x == 0 && y ==0){
				delete((NetworkTile*)tiles[x][y]);
			}else{
				delete((ProcessingTile*)tiles[x][y]);
			}
			
		}
	}
	
	if(_status)
		std::cout << "Simulation failed!"	 << std::endl;
	else 
		std::cout << "Simulation ended without errors."	 << std::endl;
	
	return _status;
}




