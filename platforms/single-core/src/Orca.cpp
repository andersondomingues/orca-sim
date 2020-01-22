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

//orca-specific hardware
#include <ProcessingTile.h>

//instantiates a mesh of MxN PE
ProcessingTile* tile;

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
	std::cout << "ORCA_NOC_WIDTH set to " << ORCA_NOC_WIDTH << std::endl;
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
	
	//create new tile (single)
	tile = new ProcessingTile();
	
	//load bin into memory
	tile->GetMem0()->LoadBin(std::string(argv[1]), MEM0_BASE, MEM0_SIZE);
	
	std::cout << "==============[ SIMULATION ]" << std::endl;
	
	//instantiate simulation
	Simulator* s = new Simulator();
		
	std::cout << "Scheduling..."	 << std::endl;
	
	//schedule pcore
	s->Schedule(Event(3, tile->GetCpu()));

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

	//CPU statuses
	std::cout 
				<< tile->GetName() 
				<< ": INTR=" << (int)(tile->GetSignalIntr()->Read()) 
				<< ", STALL=" << (int)(tile->GetSignalStall()->Read()) << std::endl;

	delete(s);
	delete(tile);
			
	if(_status)
		std::cout << "Simulation failed!"	 << std::endl;
	else 
		std::cout << "Simulation ended without errors."	 << std::endl;
	
	return _status;
}




