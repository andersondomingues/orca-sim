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
	_sig_stall      = new Signal<uint8_t>(SIGNAL_CPU_STALL, this->GetName() + ".stall");
	_sig_dma_prog   = new Signal<uint8_t>(SIGNAL_DMA_PROG, this->GetName() + ".dma_prog");
	// dummy signal required by the cpu
	_sig_intr       = new Signal<uint8_t>(SIGNAL_CPU_INTR,  this->GetName() + ".intr");

	//DMA data signals
	_sig_burst_size  = new Signal<uint32_t>(DMA_BURST_SIZE, this->GetName() + ".burst_size");
	_sig_nn_size     = new Signal<uint32_t>(DMA_NN_SIZE, this->GetName() + ".weight_mem_addr");
	_sig_out_size    = new Signal<uint32_t>(DMA_OUT_SIZE, this->GetName() + ".input_mem_addr");

	//create a cpu and memory in addition to current tile hardware
	_mem0  = new Memory(this->GetName() + ".mem0", MEM0_SIZE, MEM0_BASE); //main
	_cpu   = new HFRiscV(this->GetName() + ".cpu", _sig_intr, _sig_stall, _mem0);

	// configurable DMA controller which is able to feed multiple MACs in parallel
	_dma  = new TDmaMult(this->GetName() + ".dma_mult", _sig_stall, _sig_dma_prog, _sig_burst_size,
				 _sig_nn_size, _sig_out_size, DMA_MAC_OUT_ARRAY, _mem0);

	//bind control signals to hardware (cpu side)
	_sig_stall->MapTo(_mem0->GetMap(SIGNAL_CPU_STALL), SIGNAL_CPU_STALL);
	_sig_dma_prog->MapTo(_mem0->GetMap(SIGNAL_DMA_PROG), SIGNAL_DMA_PROG);
	_sig_intr->MapTo(_mem0->GetMap(SIGNAL_CPU_INTR), SIGNAL_CPU_INTR);

	_sig_burst_size->MapTo(_mem0->GetMap(DMA_BURST_SIZE), DMA_BURST_SIZE);
	_sig_nn_size->MapTo(_mem0->GetMap(DMA_NN_SIZE), DMA_NN_SIZE);
	_sig_out_size->MapTo(_mem0->GetMap(DMA_OUT_SIZE), DMA_OUT_SIZE);

	#ifdef MEMORY_ENABLE_COUNTERS
	//map memory counters to memory space
	_mem0->GetSignalCounterStore()->MapTo(_mem0->GetMap(M0_COUNTER_STORE_ADDR), M0_COUNTER_STORE_ADDR);
	_mem0->GetSignalCounterLoad()->MapTo(_mem0->GetMap(M0_COUNTER_LOAD_ADDR), M0_COUNTER_LOAD_ADDR);
	#endif

	#ifdef HFRISCV_ENABLE_COUNTERS
	//map cpu counters to memory space
	_cpu->GetSignalCounterArith()->MapTo(_mem0->GetMap(CPU_COUNTER_ARITH_ADDR), CPU_COUNTER_ARITH_ADDR);
	_cpu->GetSignalCounterLogical()->MapTo(_mem0->GetMap(CPU_COUNTER_LOGICAL_ADDR), CPU_COUNTER_LOGICAL_ADDR);
	_cpu->GetSignalCounterShift()->MapTo(_mem0->GetMap(CPU_COUNTER_SHIFT_ADDR), CPU_COUNTER_SHIFT_ADDR);
	_cpu->GetSignalCounterBranches()->MapTo(_mem0->GetMap(CPU_COUNTER_BRANCHES_ADDR), CPU_COUNTER_BRANCHES_ADDR);
	_cpu->GetSignalCounterJumps()->MapTo(_mem0->GetMap(CPU_COUNTER_JUMPS_ADDR), CPU_COUNTER_JUMPS_ADDR);
	_cpu->GetSignalCounterLoadStore()->MapTo(_mem0->GetMap(CPU_COUNTER_LOADSTORE_ADDR), CPU_COUNTER_LOADSTORE_ADDR);
	_cpu->GetSignalCounterCyclesTotal()->MapTo(_mem0->GetMap(CPU_COUNTER_CYCLES_TOTAL_ADDR), CPU_COUNTER_CYCLES_TOTAL_ADDR);
	_cpu->GetSignalCounterCyclesStall()->MapTo(_mem0->GetMap(CPU_COUNTER_CYCLES_STALL_ADDR), CPU_COUNTER_CYCLES_STALL_ADDR);
	_cpu->GetSignalHostTime()->MapTo(_mem0->GetMap(CPU_COUNTER_HOSTTIME_ADDR), CPU_COUNTER_HOSTTIME_ADDR);
	#endif

	this->Reset();
}

ProcessingTile::~ProcessingTile(){
	delete(_cpu);
	delete(_mem0);
	delete(_dma);

	//delete signals 
	delete(_sig_stall);
	delete(_sig_dma_prog);
	delete(_sig_intr);
	delete(_sig_burst_size);
	delete(_sig_nn_size);
	delete(_sig_out_size);
}

void ProcessingTile::Reset(){
	//reset control wires
	_sig_stall->Write(0);
	_sig_dma_prog ->Write(0);
	_sig_intr->Write(0);

	//DMA data signals
	_sig_burst_size->Write(0);
	_sig_nn_size->Write(0);
	_sig_out_size->Write(0);
}

HFRiscV* ProcessingTile::GetCpu(){
	return _cpu;
} 
TDmaMult* ProcessingTile::GetDma(){
	return _dma;
}

/************************************* GETTERS **************************************/
Signal<uint8_t>*  ProcessingTile::GetSignalStall(){ return _sig_stall; }
Signal<uint8_t>*  ProcessingTile::GetSignalDmaProg(){ return _sig_dma_prog; }
Signal<uint8_t>*  ProcessingTile::GetSignalIntr(){ return _sig_intr; }


Signal<uint32_t>* ProcessingTile::GetSignalHostTime(){
	return _signal_hosttime;
}

Memory* ProcessingTile::GetMem0(){	return _mem0;}

std::string ProcessingTile::ToString(){
	stringstream ss;
	ss << this->GetName() << "={" << _cpu->GetName() <<"}";
	return ss.str();
}

std::string ProcessingTile::GetName(){
	return "core!";
}
