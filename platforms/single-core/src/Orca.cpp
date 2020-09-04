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

//orca-specific hardware
#include <MemoryMap.h>

//interrupt signal catcher
static volatile sig_atomic_t interruption = 0;

/**
 * @brief Signal handler. This handler captures
 * interruption from the keyboard (CTRL+C) and
 * flag the simulation to end in the current
 * epoch. If pressed CTRL+C again, simulation will
 * abort.
 * @param _ This param is unused (must be here to
 * comply with system's API)
 */
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
	}	
}

/**
 * @brief Main routine. Instantiate simulator, hardware models, 
 * connect these models, and start simulation.
 * @param argc Should be always equals 2
 * @param argv One-dimensional array containing the name of binary file 
 * to load in the program
 * @return int Simulation status (termination)
 */
int main(int __attribute__((unused)) argc, char** argv){

	//show usage and abort if an image file is not informed
	if(argc != 2){
		std::cout << "Usage: " << argv[0] << " <software-image>" << std::endl;
		abort();
	}

	//register interruption handler
	signal(SIGINT, sig_handler);

	std::cout << "URSA/ORCA Platform " << std::endl;

	//create new control signals
	Signal<uint8_t> *signal_stall, *signal_intr;
	signal_stall = new Signal<uint8_t>(SIGNAL_CPU_STALL, "cpu.stall");
	signal_intr  = new Signal<uint8_t>(SIGNAL_CPU_INTR,  "cpu.intr");
	
	//create a cpu and memory 
	Memory* mem = new Memory("main-memory", MEM_SIZE, MEM_BASE);
	HFRiscV* cpu = new HFRiscV("cpu", signal_intr, signal_stall, mem);

	//bind control signals to memory space
	signal_stall->MapTo(mem->GetMap(SIGNAL_CPU_STALL), SIGNAL_CPU_STALL);
	signal_intr->MapTo(mem->GetMap(SIGNAL_CPU_INTR), SIGNAL_CPU_INTR);

	//reset control wires
	signal_stall->Write(0);
	signal_intr->Write(0);

	//map counters to memory if hardware counters were enabled
	#ifdef MEMORY_ENABLE_COUNTERS
	//map main memory counter
	mem->GetSignalCounterStore()->MapTo(mem->GetMap(M0_COUNTER_STORE_ADDR), M0_COUNTER_STORE_ADDR);
	mem->GetSignalCounterLoad()->MapTo(mem->GetMap(M0_COUNTER_LOAD_ADDR), M0_COUNTER_LOAD_ADDR);
	#endif

	#ifdef HFRISCV_ENABLE_COUNTERS
	//memory mapping
	cpu->GetSignalCounterArith()->MapTo(mem->GetMap(CPU_COUNTER_ARITH_ADDR), CPU_COUNTER_ARITH_ADDR);
	cpu->GetSignalCounterLogical()->MapTo(mem->GetMap(CPU_COUNTER_LOGICAL_ADDR), CPU_COUNTER_LOGICAL_ADDR);
	cpu->GetSignalCounterShift()->MapTo(mem->GetMap(CPU_COUNTER_SHIFT_ADDR), CPU_COUNTER_SHIFT_ADDR);
	cpu->GetSignalCounterBranches()->MapTo(mem->GetMap(CPU_COUNTER_BRANCHES_ADDR), CPU_COUNTER_BRANCHES_ADDR);
	cpu->GetSignalCounterJumps()->MapTo(mem->GetMap(CPU_COUNTER_JUMPS_ADDR), CPU_COUNTER_JUMPS_ADDR);
	cpu->GetSignalCounterLoadStore()->MapTo(mem->GetMap(CPU_COUNTER_LOADSTORE_ADDR), CPU_COUNTER_LOADSTORE_ADDR);
	cpu->GetSignalCounterCyclesTotal()->MapTo(mem->GetMap(CPU_COUNTER_CYCLES_TOTAL_ADDR), CPU_COUNTER_CYCLES_TOTAL_ADDR);
	cpu->GetSignalCounterCyclesStall()->MapTo(mem->GetMap(CPU_COUNTER_CYCLES_STALL_ADDR), CPU_COUNTER_CYCLES_STALL_ADDR);
	cpu->GetSignalHostTime()->MapTo(mem->GetMap(CPU_COUNTER_HOSTTIME_ADDR), CPU_COUNTER_HOSTTIME_ADDR);
	#endif

	//load software image into memory
	mem->LoadBin(std::string(argv[1]), MEM_BASE, MEM_SIZE);
	
	//instantiate a new simulation
	Simulator* s = new Simulator();
		
	//schedule cpu
	s->Schedule(Event(3, cpu));

	std::cout << "Epoch set to " << ORCA_EPOCH_LENGTH << " cycles." << std::endl;
	std::cout << "Please wait..." << std::endl;

	try{
	
		std::chrono::high_resolution_clock::time_point t1, t2;
	
		while(!interruption && !cpu->GetState()->terminated){
			
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
		return -1; //abnormal termination, simulation failed
	}

	//minimal reporting
	std::cout << "cpu: INTR=" << (int)(signal_intr->Read()) 
				<< ", STALL=" << (int)(signal_stall->Read()) << std::endl;

	//if counters enabled, report them
	#ifdef HFRISCV_ENABLE_COUNTERS
	std::cout << "cpu"
		"\tiarith\t\t" << (int)cpu->GetSignalCounterArith()->Read() << std::endl <<
		"\tilogic\t\t" << (int)cpu->GetSignalCounterLogical()->Read() << std::endl <<
		"\tishift\t\t" << (int)cpu->GetSignalCounterShift()->Read() << std::endl <<
		"\tibranch\t\t" << (int)cpu->GetSignalCounterBranches()->Read() << std::endl <<
		"\tijumps\t\t" << (int)cpu->GetSignalCounterJumps()->Read() << std::endl <<
		"\timemop\t\t" << (int)cpu->GetSignalCounterLoadStore()->Read() << std::endl <<
		"\ticycles\t\t" << (int)cpu->GetSignalCounterCyclesTotal()->Read() << std::endl <<
		"\tistalls\t\t" << (int)cpu->GetSignalCounterCyclesStall()->Read() << std::endl <<
		"\tihtime\t\t" << (int)cpu->GetSignalHostTime()->Read() << std::endl;

	#endif

	#ifdef MEMORY_ENABLE_COUNTERS
	std::cout << "mem"
		"\tloads\t\t" << (int)mem->GetSignalCounterStore()->Read() << std::endl <<
		"\tstores\t\t" << (int)mem->GetSignalCounterLoad()->Read() << std::endl;
	#endif

	int exit_status = cpu->GetState()->terminated;

	//free resources
	delete(s);
	delete(cpu);
	delete(mem);
	delete(signal_intr);
	delete(signal_stall);
	
	//return existing code to upper system
	return exit_status;
}




