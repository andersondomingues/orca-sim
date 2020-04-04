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

	//ni sig wires
	_signal_stall       = new USignal<uint8_t>(SIGNAL_CPU_STALL, this->GetName() + ".stall");
	_signal_intr        = new USignal<uint8_t>(SIGNAL_CPU_INTR,  this->GetName() + ".intr");
	_signal_send_status = new USignal<uint8_t>(SIGNAL_SEND_STATUS, this->GetName() + ".send_status");
	_signal_recv_status = new USignal<uint32_t>(SIGNAL_RECV_STATUS, this->GetName() + ".recv_status");
	_signal_prog_send   = new USignal<uint8_t>(SIGNAL_PROG_SEND, this->GetName() + ".progr_send");
	_signal_prog_recv   = new USignal<uint8_t>(SIGNAL_PROG_RECV, this->GetName() + ".progr_recv");
	_signal_prog_addr   = new USignal<uint32_t>(SIGNAL_PROG_ADDR, this->GetName() + ".progr_addr");
	_signal_prog_size   = new USignal<uint32_t>(SIGNAL_PROG_SIZE, this->GetName() + ".progr_size");

	//create a cpu and memory in addition to current tile hardware
	_mem0  = new UMemory(this->GetName() + ".mem0", MEM0_SIZE, MEM0_BASE); //main
	_cpu   = new THellfireProcessor(this->GetName() + ".cpu", _signal_intr, _signal_stall);
	_netif  = new TDmaNetif (this->GetName() + ".netif");
	
	//binds cpu to the main memory
	_cpu->SetMem0(_mem0);
	_netif->SetMem0(_mem0);
	
	//reset control wires
    _signal_stall->Write(0);
	_signal_intr->Write(0); 
	
	_signal_send_status->Write(0);
	_signal_recv_status->Write(0);
	_signal_prog_send->Write(0);
	_signal_prog_recv->Write(0);
	_signal_prog_addr->Write(0);
	_signal_prog_size->Write(0);
		
	//bind control signals to hardware (netif side)
	_netif->SetSignalStall(_signal_stall);
	_netif->SetSignalIntr(_signal_intr);
	_netif->SetSignalSendStatus(_signal_send_status);
	_netif->SetSignalRecvStatus(_signal_recv_status);
	_netif->SetSignalProgSend(_signal_prog_send);
	_netif->SetSignalProgRecv(_signal_prog_recv);
	_netif->SetSignalProgAddr(_signal_prog_addr);
	_netif->SetSignalProgSize(_signal_prog_size);
	
	//bind netif to router
	this->GetRouter()->SetOutputBuffer(_netif->GetInputBuffer(), LOCAL);
	_netif->SetOutputBuffer(this->GetRouter()->GetInputBuffer(LOCAL));

	//create new memories for the NI
	_mem1 = new UMemory(this->GetName() + ".mem1", MEM1_SIZE, 0); //read from noc 
	_mem2 = new UMemory(this->GetName() + ".mem2", MEM2_SIZE, 0); //write to noc

	//bind memories
	_netif->SetMem1(_mem1);	
	_netif->SetMem2(_mem2);

	//bind self-id wire (care to save the value before the bind)
	this->GetSignalId()->MapTo(_mem0->GetMap(MAGIC_TILE_ID), MAGIC_TILE_ID);
	
	//update naming of internal hardware parts (from internal class)
	this->GetRouter()->SetName(this->GetName() + ".router");
	this->GetDmaNetif()->SetName(this->GetName() + ".netif");
	this->GetMem1()->SetName(this->GetName() + ".mem1");
	this->GetMem2()->SetName(this->GetName() + ".mem2");

	//bind control signals to hardware (cpu side)
	this->GetSignalStall()->MapTo(_mem0->GetMap(SIGNAL_CPU_STALL), SIGNAL_CPU_STALL);
	this->GetSignalIntr()->MapTo(_mem0->GetMap(SIGNAL_CPU_INTR), SIGNAL_CPU_INTR);
	this->GetSignalSendStatus()->MapTo(_mem0->GetMap(SIGNAL_SEND_STATUS), SIGNAL_SEND_STATUS);
	this->GetSignalRecvStatus()->MapTo(_mem0->GetMap(SIGNAL_RECV_STATUS), SIGNAL_RECV_STATUS);

	this->GetSignalProgSend()->MapTo(_mem0->GetMap(SIGNAL_PROG_SEND), SIGNAL_PROG_SEND);
	this->GetSignalProgRecv()->MapTo(_mem0->GetMap(SIGNAL_PROG_RECV), SIGNAL_PROG_RECV);
	
	this->GetSignalProgAddr()->MapTo(_mem0->GetMap(SIGNAL_PROG_ADDR), SIGNAL_PROG_ADDR);
	this->GetSignalProgSize()->MapTo(_mem0->GetMap(SIGNAL_PROG_SIZE), SIGNAL_PROG_SIZE);

	#ifdef MEMORY_ENABLE_COUNTERS
	//map main memory counter
	_mem0->InitCounters(M0_COUNTER_STORE_ADDR, M0_COUNTER_LOAD_ADDR);
	_mem0->GetSignalCounterStore()->MapTo(_mem0->GetMap(M0_COUNTER_STORE_ADDR), M0_COUNTER_STORE_ADDR);
	_mem0->GetSignalCounterLoad()->MapTo(_mem0->GetMap(M0_COUNTER_LOAD_ADDR), M0_COUNTER_LOAD_ADDR);

	//map secondary memory counters
	_mem1->InitCounters(M1_COUNTER_STORE_ADDR, M1_COUNTER_LOAD_ADDR);
	_mem1->GetSignalCounterStore()->MapTo(_mem0->GetMap(M1_COUNTER_STORE_ADDR), M1_COUNTER_STORE_ADDR);
	_mem1->GetSignalCounterLoad()->MapTo(_mem0->GetMap(M1_COUNTER_LOAD_ADDR), M1_COUNTER_LOAD_ADDR);

	_mem2->InitCounters(M2_COUNTER_STORE_ADDR, M2_COUNTER_LOAD_ADDR);
	_mem2->GetSignalCounterStore()->MapTo(_mem0->GetMap(M2_COUNTER_STORE_ADDR), M2_COUNTER_STORE_ADDR);
	_mem2->GetSignalCounterLoad()->MapTo(_mem0->GetMap(M2_COUNTER_LOAD_ADDR), M2_COUNTER_LOAD_ADDR);
	#endif

	#ifdef ROUTER_ENABLE_COUNTERS
	//counters have been initialized by syperclass, only mapping is required
	this->GetRouter()->GetSignalCounterActive()->MapTo(_mem0->GetMap(ROUTER_COUNTER_ACTIVE_ADDR), ROUTER_COUNTER_ACTIVE_ADDR, 0);
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
}

ProcessingTile::~ProcessingTile(){

	delete(_cpu);
	delete(_mem0);
	
	delete(_netif);
	delete(_mem1);
	delete(_mem2);
	
	//delete signals 
	delete(_signal_stall);
	delete(_signal_intr);
	delete(_signal_send_status);
	delete(_signal_recv_status);
	delete(_signal_prog_send);
	delete(_signal_prog_recv);
	delete(_signal_prog_addr);
	delete(_signal_prog_size);	
}

THellfireProcessor* ProcessingTile::GetCpu(){
	return _cpu;
} 


/**
 * @brief Get current NI module
 * @return A pointer to the instance of NI
 */
TDmaNetif*  ProcessingTile::GetDmaNetif(){
	return _netif;
}

/**
 * @brief Get sender memory module 
 * @return A pointer to the instance of memory
 */
UMemory* ProcessingTile::GetMem1(){
	return _mem1;
}

/**
 * @brief Get recv memory module
 * @return A pointer to the instance of memory
 */
UMemory* ProcessingTile::GetMem2(){
	return _mem2;
}

/************************************* GETTERS **************************************/
USignal<uint8_t>*  ProcessingTile::GetSignalStall(){ return _signal_stall; }
USignal<uint8_t>*  ProcessingTile::GetSignalIntr(){ return _signal_intr; }

USignal<uint8_t>*  ProcessingTile::GetSignalSendStatus(){ return _signal_send_status; }
USignal<uint32_t>*  ProcessingTile::GetSignalRecvStatus(){ return _signal_recv_status; }

USignal<uint8_t>*  ProcessingTile::GetSignalProgSend(){ return _signal_prog_send; }
USignal<uint8_t>*  ProcessingTile::GetSignalProgRecv(){ return _signal_prog_recv; }

USignal<uint32_t>* ProcessingTile::GetSignalProgAddr(){ return _signal_prog_addr; }
USignal<uint32_t>* ProcessingTile::GetSignalProgSize(){ return _signal_prog_size; }

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
