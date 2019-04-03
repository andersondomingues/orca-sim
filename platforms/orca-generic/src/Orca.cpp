/** 
 * This file is part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
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
Tile* tiles[NOC_W_SIZE][NOC_H_SIZE];

#ifndef OPT_HFRISC_DISABLE_METRICS
/**
 * Report Metrics for CPUs
 * @param metric Metric which to report
 * @param unit String representation of measurement unit (for printing purpose)
*/ 
void report_metrics_cpu(){

	ProcessingTile* t;
	Metric* m;
	
	//prints header
	std::cout << "==============[ CPU POWER/ENERGY STATISTICS ]" << std::endl;
	printf("(x,y)\tPower(mW)\n");
	
	//show simulation statistics
	
	for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){
			
			//network interface has no caracterization
			if(i == 0 && j ==0) continue;
			
			//get current processing tile
			t = (ProcessingTile*)tiles[i][j];
			
			//print position on noc
			printf("(%d,%d)\t", i, j);
			
			//print estimated energy
			m = t->GetCpu()->GetMetric(Metrics::AVG_POWER_DYNAMIC); //mW
			printf("%.6f\t", m->GetAccumulative()); //pJ -> nJ (pico to nano)
			
			printf("\n");
			
			/*
				
			_power_dynamic = t->GetCpu()->GetMetric(Metrics::AVG_POWER_DYNAMIC);
	
			printf("S:\t%d\t %d\t %d\t %d\t\n",
				_power_dynamic->GetSamples(),
				_power_leakage->GetSamples(),
				_energy_dynamic->GetSamples(),
				_energy_leakage->GetSamples()			
			);
			 */
		}
	}
}
#endif


void connect_routers(TRouter* r1, uint32_t p1, TRouter* r2, uint32_t p2){
	r1->SetOutputBuffer(r2->GetInputBuffer(p2), p1);
	r2->SetOutputBuffer(r1->GetInputBuffer(p1), p2);
	
	std::cout << "router_comm: " << r1->GetName() << " ----[" 
			  << p1 << "/" << p2 << "]---- " << r2->GetName() << std::endl;
}



int main(int argc, char** argv){

    argc = argc; //workaround to use -Wextra

	std::cout << "ORCA Platform (Width=" << NOC_W_SIZE << ", Height=" << NOC_H_SIZE << ")" << std::endl;
	std::cout << "Simulation step set to " << CYCLES_TO_SIM << " cycles." << std::endl;
	std::cout << "Instanting new hardware..." << std::endl;
	
	std::cout << "==============[ TILE LIST ]" << std::endl;
	
	//populate tiles
	for(int x = 0; x < NOC_W_SIZE; x++){
		for(int y = 0; y < NOC_H_SIZE; y++){
			
			if(x == 0 && y ==0){
				std::cout << "(0,0) is a network tile" << std::endl;
				tiles[x][y] = (Tile*)new NetworkTile(x, y);				
			}else{
				std::cout << "(" << x << "," << y << ") is a processing tile" << std::endl;
				tiles[x][y] = (Tile*)new ProcessingTile(x, y);
			}
		}
	}
	
	std::cout << "==============[ ROUTER CONNECTIONS ]" << std::endl;
	
	//connect tiles to each other (left-to-right, right-to-left connections)	
	for(int x = 0; x < NOC_W_SIZE - 1; x++)
		for(int y = 0; y < NOC_H_SIZE; y++)
			connect_routers(tiles[x][y]->GetRouter(), EAST, tiles[x+1][y]->GetRouter(), WEST);
			
	//connect tiles to each other (bottom-to-top, top-to-bottom connections)
	for(int x = 0; x < NOC_W_SIZE; x++)
		for(int y = 0; y < NOC_H_SIZE - 1; y++)
			connect_routers(tiles[x][y]->GetRouter(), NORTH, tiles[x][y+1]->GetRouter(), SOUTH);
			
	//load binaries into main memories (processing tiles only)
	for(int x = 0; x < NOC_W_SIZE; x++){
		for(int y = 0; y < NOC_H_SIZE; y++){
			
			//zero-zero is for network interface
			if(x == 0 && y == 0) continue;
		
			//index = x + NOC_W_SIZE * y;
			((ProcessingTile*)tiles[x][y])->GetMem0()->LoadBin(std::string(argv[1]), MEM0_BASE, MEM0_SIZE);
		}
	}

	std::cout << "==============[ SIMULATION ]" << std::endl;
	
	//instantiate simulation
	Simulator* s = new Simulator();
		
	std::cout << "Scheduling..."	 << std::endl;
	
	//schedule hardware to be simulated
	for(int x = 0; x < NOC_W_SIZE; x++){
		for(int y = 0; y < NOC_H_SIZE; y++){
			
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

	std::cout << "Simulation pulse set to " << CYCLES_TO_SIM << " cycles. Please wait..." << std::endl;

	//keep simulating until something happen
	uint32_t gigacycles = 0;
	try{
		//while(1){
			
			std::chrono::high_resolution_clock::time_point t1 = 
				std::chrono::high_resolution_clock::now();
			
			gigacycles++;
			s->Run(CYCLES_TO_SIM);			
			
			std::chrono::high_resolution_clock::time_point t2 =
				std::chrono::high_resolution_clock::now();
				
			auto duration = 
				std::chrono::duration_cast
				<std::chrono::milliseconds>( t2 - t1 ).count();
			
			//converts mili to seconds before calculating the frequency
			double hertz = CYCLES_TO_SIM / (double)(duration / 1000);
			
			//divide frequency by 1k (Hz -> KHz)
			std::cout << "notice: pulse #" << gigacycles << " took " 
				<< (duration / 1000.0)<< " seconds @ "
				<< (hertz / 1000.0) <<" KHz)" << std::endl;
		//}
		
	}catch(std::runtime_error& e){
		std::cout << e.what() << std::endl;
		goto clean;
	}
	
	std::cout << "Simulation ended without errors."	 << std::endl;
	
	//show buffer status
	std::cout << "==============[ BUFFERS' STATUSES ]" << std::endl;
	for(int x = 0; x < NOC_W_SIZE; x++){
		for(int y = 0; y < NOC_H_SIZE; y++){
		
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
	std::cout << "==============[ NI' STATUSES ]" << std::endl;
	for(int x = 0; x < NOC_W_SIZE; x++){
		for(int y = 0; y < NOC_H_SIZE; y++){
		
			TNetif* n = tiles[x][y]->GetNetif();
			std::cout << n->GetName() << ":"
				<< " Send=" << static_cast<int>(n->GetSendState())
				<< " Recv=" << static_cast<int>(n->GetRecvState())
				<< std::endl;
		}
	}
	
	#ifndef OPT_HFRISC_DISABLE_METRICS
	report_metrics_cpu();	
	#endif
	
	#ifndef OPT_ROUTER_DISABLE_METRICS
	std::cout << "==============[ ROUTING POWER/ENERGY STATISTICS ]" << std::endl;
	for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){
			
			if(i == 0 && j ==0) continue;
			
			Metric* energy = tiles[i][j]->GetRouter()->GetMetric(Metrics::ENERGY);
			
			std::cout << tiles[i][j]->GetRouter()->GetName() << ":"
			          << "  samples=" << std::dec << energy->GetSamples() 
					  << "  acc.=" << setprecision(4) << energy->GetAccumulative() << "uW"
					  << "  avg.=" << (energy->GetAccumulative() / energy->GetSamples()) << "uW"<< std::endl;
		}
		
	}
	#endif
	
	return 0;
	
clean:
	
	delete(s); //sim
	
	//delete PE
	for(int x = 0; x < NOC_W_SIZE; x++)
		for(int y = 0; y < NOC_H_SIZE; y++)
			delete(tiles[x][y]);

	return 1;	
}
