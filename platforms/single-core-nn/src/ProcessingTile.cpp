/** 
 * This file is part of project URSA. More information on the project
 * can be found at URSA's repository at GitHub
 * 
 * http://https://github.com/andersondomingues/ursa
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
#include <sstream>

//model API
#include <ProcessingTile.h>

/** 
 * Default constructor.
 * Instantiate and bind internal hardware to each
 * other. */
ProcessingTile::ProcessingTile() {
	//DMA control signals
	_sig_stall      = new USignal<uint8_t>(SIGNAL_CPU_STALL, this->GetName() + ".stall");
	_sig_dma_prog   = new USignal<uint8_t>(SIGNAL_DMA_PROG, this->GetName() + ".dma_prog");
	// dummy signal required by the cpu
	_sig_intr       = new USignal<uint8_t>(SIGNAL_CPU_INTR,  this->GetName() + ".intr");

	//DMA data signals
	_sig_burst_size      = new USignal<uint32_t>(DMA_BURST_SIZE, this->GetName() + ".burst_size");
	_sig_weight_mem_addr = new USignal<uint32_t>(DMA_WEIGHT_MEM_ADDR, this->GetName() + ".weight_mem_addr");
	_sig_input_mem_addr  = new USignal<uint32_t>(DMA_INPUT_MEM_ADDR, this->GetName() + ".input_mem_addr");
	_sig_mac_out         = new USignal<uint32_t>(DMA_MAC_OUT, this->GetName() + ".mac_out");

	//create a cpu and memory in addition to current tile hardware
	_mem0  = new UMemory(this->GetName() + ".mem0", MEM0_SIZE, MEM0_BASE); //main
	_cpu   = new THellfireProcessor(this->GetName() + ".cpu", _sig_intr, _sig_stall);
	
	//binds cpu to vetorial sequential multipliers	
	TimedFPMultiplier* auxMult;
	for(int i=0;i<SIMD_SIZE;i++){
		auxMult = new TimedFPMultiplier(this->GetName() + ".seq_mult_vet["+std::to_string(i)+"]");
		_seqMultVet.push_back(auxMult);
		_cpu->SetSeqMultVet(_seqMultVet[i]);
	}

	// TODO create the mult and memW/memI in a loop controlled by SIMD_SIZE
	//Timed multiplier controller
	//_memW  = new UMemory(this->GetName() + ".memW", NN_MEM_SIZE, MEMW_BASE); //weight memory
	//_memI  = new UMemory(this->GetName() + ".memI", NN_MEM_SIZE, MEMI_BASE); //input  memory
	_memW = new USignal<uint32_t>(MEMW_BASE, this->GetName() + ".memW[0]");
	_memI = new USignal<uint32_t>(MEMI_BASE, this->GetName() + ".memI[0]");
	_dma  = new TDmaMult(this->GetName() + ".dma_mult", _sig_stall, _sig_dma_prog, _sig_burst_size,
				 _sig_weight_mem_addr, _sig_input_mem_addr, _sig_mac_out, _memW, _memI, NN_MEM_BANK_HEIGHT,
				 _seqMultVet[0]);
	//binds cpu to the main memory
	_cpu->SetMem0(_mem0);   

	//bind control signals to hardware (cpu side)
	this->GetSignalStall()->MapTo((uint8_t*)_mem0->GetMap(SIGNAL_CPU_STALL), SIGNAL_CPU_STALL);
	this->GetSignalIntr()->MapTo((uint8_t*)_mem0->GetMap(SIGNAL_CPU_INTR), SIGNAL_CPU_INTR);

	#ifdef MEMORY_ENABLE_COUNTERS
	//map main memory counter
	_mem0->InitCounters(M0_COUNTER_STORE_ADDR, M0_COUNTER_LOAD_ADDR);
	_mem0->GetSignalCounterStore()->MapTo(
		(uint32_t*)(_mem0->GetMap(M0_COUNTER_STORE_ADDR)), M0_COUNTER_STORE_ADDR);
	_mem0->GetSignalCounterLoad()->MapTo(
		(uint32_t*)(_mem0->GetMap(M0_COUNTER_LOAD_ADDR)), M0_COUNTER_LOAD_ADDR);
	#endif

	//----------------- initialize counters for the cpu
	#ifdef HFRISCV_ENABLE_COUNTERS
	_cpu->InitCounters(
		CPU_COUNTER_ARITH_ADDR, CPU_COUNTER_LOGICAL_ADDR, CPU_COUNTER_SHIFT_ADDR, 
		CPU_COUNTER_BRANCHES_ADDR, CPU_COUNTER_JUMPS_ADDR, CPU_COUNTER_LOADSTORE_ADDR,
		CPU_COUNTER_CYCLES_TOTAL_ADDR, CPU_COUNTER_CYCLES_STALL_ADDR,
		CPU_COUNTER_HOSTTIME_ADDR
	);

	//memory mapping
	_cpu->GetSignalCounterArith()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_ARITH_ADDR)), CPU_COUNTER_ARITH_ADDR);
	_cpu->GetSignalCounterLogical()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_LOGICAL_ADDR)), CPU_COUNTER_LOGICAL_ADDR);
	_cpu->GetSignalCounterShift()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_SHIFT_ADDR)), CPU_COUNTER_SHIFT_ADDR);
	_cpu->GetSignalCounterBranches()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_BRANCHES_ADDR)), CPU_COUNTER_BRANCHES_ADDR);
	_cpu->GetSignalCounterJumps()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_JUMPS_ADDR)), CPU_COUNTER_JUMPS_ADDR);
	_cpu->GetSignalCounterLoadStore()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_LOADSTORE_ADDR)), CPU_COUNTER_LOADSTORE_ADDR);
	_cpu->GetSignalCounterCyclesTotal()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_CYCLES_TOTAL_ADDR)), CPU_COUNTER_CYCLES_TOTAL_ADDR);
	_cpu->GetSignalCounterCyclesStall()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_CYCLES_STALL_ADDR)), CPU_COUNTER_CYCLES_STALL_ADDR);
	_cpu->GetSignalHostTime()->MapTo(
		(uint32_t*)(_mem0->GetMap(CPU_COUNTER_HOSTTIME_ADDR)), CPU_COUNTER_HOSTTIME_ADDR);
	#endif

	this->Reset();
}

ProcessingTile::~ProcessingTile(){
	delete(_cpu);
	delete(_mem0);
	delete(_memW);
	delete(_memI);
	delete(_dma);

	//delete signals 
	delete(_sig_stall);
	delete(_sig_dma_prog);
	delete(_sig_intr);
	delete(_sig_burst_size);
	delete(_sig_weight_mem_addr);
	delete(_sig_input_mem_addr);
	delete(_sig_mac_out);
}

void ProcessingTile::Reset(){

	//reset control wires
	_sig_stall->Write(0);
	_sig_dma_prog ->Write(0);
	_sig_intr->Write(0);

	//DMA data signals
	_sig_burst_size->Write(0);
	_sig_weight_mem_addr->Write(0);
	_sig_input_mem_addr->Write(0);
	_sig_mac_out->Write(0);  
}

THellfireProcessor* ProcessingTile::GetCpu(){
	return _cpu;
} 

/************************************* GETTERS **************************************/
USignal<uint8_t>*  ProcessingTile::GetSignalStall(){ return _sig_stall; }
USignal<uint8_t>*  ProcessingTile::GetSignalDmaProg(){ return _sig_dma_prog; }
USignal<uint8_t>*  ProcessingTile::GetSignalIntr(){ return _sig_intr; }

/**
 * @brief Get current signal for systime signal
 * @return A pointer to the instance of signal
 */
USignal<uint32_t>* ProcessingTile::GetSignalHostTime(){
	return _signal_hosttime;
}

UMemory* ProcessingTile::GetMem0(){	return _mem0;}
USignal<uint32_t>* ProcessingTile::GetMemW(){	return _memW;}
USignal<uint32_t>* ProcessingTile::GetMemI(){	return _memI;}

TimedFPMultiplier* ProcessingTile::GetSeqMultVet(int idx){
	return _seqMultVet[idx];
}

std::string ProcessingTile::ToString(){
	stringstream ss;
	ss << this->GetName() << "={" << _cpu->GetName() <<"}";
	
	return ss.str();
}

std::string ProcessingTile::GetName(){
	
	return "core!";
}
