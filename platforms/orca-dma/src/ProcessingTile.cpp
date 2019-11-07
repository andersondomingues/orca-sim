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
#include <THellfireProcessor.h>
#include <TDmaNetif.h>
#include <TRouter.h>
#include <UMemory.h>

#include <ProcessingTile.h>

/** 
 * Default constructor.
 * Instantiate and bind internal hardware to each
 * other. */
ProcessingTile::ProcessingTile(uint32_t x, uint32_t y) : Tile(x, y) {

	//create a cpu and memory in addition to current tile hardware
	_mem0 = new UMemory(this->GetName() + ".mem0", MEM0_SIZE, MEM0_BASE); //main
	_cpu  = new THellfireProcessor(this->GetName() + ".cpu", this->GetSignalIntr(), this->GetSignalStall());
	
	//binds cpu to the main memory
	_cpu->SetMem0(_mem0);
	this->GetDmaNetif()->SetMem0(_mem0);
	
	//bind self-id wire (care to save the value before the bind)
	this->GetSignalId()->MapTo((uint32_t*)(_mem0->GetMap(MAGIC_TILE_ID)), MAGIC_TILE_ID);
	
	//update naming of internal hardware parts (from internal class)
	this->GetRouter()->SetName(this->GetName() + ".router");
	this->GetDmaNetif()->SetName(this->GetName() + ".netif");
	this->GetMem1()->SetName(this->GetName() + ".mem1");
	this->GetMem2()->SetName(this->GetName() + ".mem2");

	//bind control signals to hardware (cpu side)
	this->GetSignalStall()->MapTo(_mem0->GetMap(SIGNAL_CPU_STALL), SIGNAL_CPU_STALL);
	this->GetSignalIntr()->MapTo(_mem0->GetMap(SIGNAL_CPU_INTR), SIGNAL_CPU_INTR);
	this->GetSignalSendStatus()->MapTo(_mem0->GetMap(SIGNAL_SEND_STATUS), SIGNAL_SEND_STATUS);
	this->GetSignalRecvStatus()->MapTo((int32_t*)_mem0->GetMap(SIGNAL_RECV_STATUS), SIGNAL_RECV_STATUS);

	this->GetSignalProgSend()->MapTo(_mem0->GetMap(SIGNAL_PROG_SEND), SIGNAL_PROG_SEND);
	this->GetSignalProgRecv()->MapTo(_mem0->GetMap(SIGNAL_PROG_RECV), SIGNAL_PROG_RECV);
	
	this->GetSignalProgAddr()->MapTo((int32_t*)(_mem0->GetMap(SIGNAL_PROG_ADDR)), SIGNAL_PROG_ADDR);
	this->GetSignalProgSize()->MapTo((int32_t*)(_mem0->GetMap(SIGNAL_PROG_SIZE)), SIGNAL_PROG_SIZE);

	#ifdef MEMORY_ENABLE_COUNTERS
	//map main memory counter
	_mem0->InitCounters(M0_COUNTER_STORE_ADDR, M0_COUNTER_LOAD_ADDR);
	_mem0->GetSignalCounterStore()->MapTo(
		(uint32_t*)(_mem0->GetMap(M0_COUNTER_STORE_ADDR)), M0_COUNTER_STORE_ADDR);
	_mem0->GetSignalCounterLoad()->MapTo(
		(uint32_t*)(_mem0->GetMap(M0_COUNTER_LOAD_ADDR)), M0_COUNTER_LOAD_ADDR);

	//map secondary memory counters (counter were initialized by superclass already, only mapping is required).
	this->GetMem1()->GetSignalCounterStore()->MapTo(
		(uint32_t*)(_mem0->GetMap(M1_COUNTER_STORE_ADDR)), M1_COUNTER_STORE_ADDR);
	this->GetMem1()->GetSignalCounterLoad()->MapTo(
		(uint32_t*)(_mem0->GetMap(M1_COUNTER_LOAD_ADDR)), M1_COUNTER_LOAD_ADDR);

	this->GetMem2()->GetSignalCounterStore()->MapTo(
		(uint32_t*)(_mem0->GetMap(M2_COUNTER_STORE_ADDR)), M2_COUNTER_STORE_ADDR);
	this->GetMem2()->GetSignalCounterLoad()->MapTo(
		(uint32_t*)(_mem0->GetMap(M2_COUNTER_LOAD_ADDR)), M2_COUNTER_LOAD_ADDR);
	#endif

	#ifdef ROUTER_ENABLE_COUNTERS
	//counters have been initialized by syperclass, only mapping is required
	this->GetRouter()->GetSignalCounterActive()->MapTo(
		(uint32_t*)_mem0->GetMap(ROUTER_COUNTER_ACTIVE_ADDR), ROUTER_COUNTER_ACTIVE_ADDR, 0);
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
}

ProcessingTile::~ProcessingTile(){

	delete(_cpu);
	delete(_mem0);
}

THellfireProcessor* ProcessingTile::GetCpu(){
	return _cpu;
} 

/**
 * @brief Get current signal for systime signal
 * @return A pointer to the instance of signal
 */
USignal<uint32_t>* ProcessingTile::GetSignalHostTime(){
	return _signal_hosttime;
}

UMemory* ProcessingTile::GetMem0(){
	return _mem0;
}

std::string ProcessingTile::ToString(){
	stringstream ss;
	ss << this->GetName() << "={" << _cpu->GetName() 
	   << ", " << this->GetRouter()->GetName() 
	   << ", " << this->GetDmaNetif()->GetName() << "}";
	
	return ss.str();
}