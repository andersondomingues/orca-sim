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
#include <Event.h>
#include <Simulator.h>

//opaque
#include <UMemory.h>

//processes
#include <THellfireProcessor.h>
#include <TRouter.h>  //hermes router
#include <TNetif.h>   //hermes ni
//#include <TDma2.h>    //hermes dma
//#include <TEth.h>    //ethernet module

#define CYCLES_TO_SIM 10000000
#define NOC_H_SIZE 3
#define NOC_W_SIZE 3

#define MEM_SIZE  0x00200000
#define SRAM_BASE 0x40000000

//function prototypes d
void printBuffers();

//objects to be simulated
TRouter* routers[NOC_W_SIZE][NOC_H_SIZE];
TNetif* netifs[NOC_W_SIZE][NOC_H_SIZE];
THellfireProcessor* cpus[NOC_W_SIZE][NOC_H_SIZE];
UMemory* mems[NOC_W_SIZE][NOC_H_SIZE];

//instantiates a mesh of MxM routers
//----------------------------------
//   (0,2)  (1,2)  (2,2)
//   
//   (0,1)  (1,1)  (2,1)
// 
//   (0,0)  (1,0)  (2,0)
//----------------------------------
void MakePes(Simulator* sptr){

	//instantiate elements
	for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){
			mems[i][j] = new UMemory("MEM_" + std::to_string(i) + "_" + std::to_string(j), MEM_SIZE, SRAM_BASE);
			netifs[i][j] = new TNetif("NETIF_" + std::to_string(i) + "_" + std::to_string(j));
			cpus[i][j] = new THellfireProcessor("HF_" + std::to_string(i) + "_" + std::to_string(j), mems[i][j], MEM_SIZE, SRAM_BASE);
			routers[i][j] = new TRouter("ROUTER_" + std::to_string(i) + "_" + std::to_string(j), i, j);
		}
	}	
	
	//bind memory to dmni
	//TODO: must bind memory to CPU and DMNI
	/*for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){
			netifs[i][j]->SetMemoryModel(mems[i][j]);
		}
	}*/
	
	//connect router to netifs
	for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){
			routers[i][j]->SetOutputBuffer(netifs[i][j]->GetInputBuffer(), LOCAL);
			netifs[i][j]->SetOutputBuffer(routers[i][j]->GetInputBuffer(LOCAL));
		}
	}
	
	//bind routers to the left 
    for(int i = 1; i < NOC_W_SIZE; i++)
        for(int j = 0; j < NOC_H_SIZE; j++)
            routers[i][j]->SetOutputBuffer(
                routers[i -1][j]->GetOutputBuffer(EAST),
                WEST
            );

    //bind routers to the right
    for(int i = 0; i < NOC_W_SIZE-1; i++)
        for(int j = 0; j < NOC_H_SIZE; j++)
            routers[i][j]->SetOutputBuffer(
                routers[i +1][j]->GetOutputBuffer(WEST),
                EAST
            );

    //bind routers to the top
    for(int i = 0; i < NOC_W_SIZE; i++)
        for(int j = 1; j < NOC_H_SIZE; j++)
            routers[i][j]->SetOutputBuffer(
                routers[i][j-1]->GetOutputBuffer(NORTH), 
                SOUTH
            );
    
    //bind routers to the bottom
    for(int i = 0; i < NOC_W_SIZE; i++)
        for(int j = 0; j < NOC_H_SIZE -1; j++)
            routers[i][j]->SetOutputBuffer(
                routers[i][j+1]->GetOutputBuffer(SOUTH), 
                NORTH
            );	
}

void loadBins(std::string path){

	int index = 0;
	//load binaries into memories
	for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){

			string code = path + "code" + std::to_string(index) + ".bin";
			std::cout << "Loading '" << code << "' to '" << mems[i][j]->GetName() << "'" << std::endl;
			mems[i][j]->LoadBin(code, SRAM_BASE, MEM_SIZE);
			index++;
		}
	}
}

int main(int argc, char** argv){

	//ptr to simulation
	Simulator* s = new Simulator();
	
	//instantiate hardware
	MakePes(s);
		
	//schedule elements
	for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){
			s->Schedule(Event(1, cpus[i][j]));
			s->Schedule(Event(1, routers[i][j]));
			s->Schedule(Event(1, netifs[i][j]));
		}
	}

	//reset everything
	for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){
			netifs[i][j]->Reset();
			cpus[i][j]->Reset();
			routers[i][j]->Reset();
		}
	}
	
	//load binaries
	std::string x = std::string(argv[1]);
	loadBins(x);
	
	std::cout << "Instantiated hardware: " << std::endl;
	//print all object names
	for(int i = 0; i < NOC_W_SIZE; i++){
		for(int j = 0; j < NOC_H_SIZE; j++){
			string pe_name = cpus[i][j]->GetName() + " | " + netifs[i][j]->GetName() + " | " + routers[i][j]->GetName();
			std::cout << pe_name << std::endl;
		}
	}

	try{
		//keep simulating until something happen
		while(1){
			s->Run(CYCLES_TO_SIM);
			std::cout << "Simulation: " << CYCLES_TO_SIM << " cycles has been passed since last message." << std::endl;
		}
	}catch(std::runtime_error& e){
		std::cout << e.what() << std::endl;
	}
	
	std::cout << std::flush;
}
