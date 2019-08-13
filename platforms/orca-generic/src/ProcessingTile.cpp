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
#include <TNetif.h>
#include <TRouter.h>
#include <UMemory.h>

#include <ProcessingTile.h>

/** 
 * Default constructor.
 * Instantiate and bind internal hardware to each
 * other. */
ProcessingTile::ProcessingTile(uint32_t x, uint32_t y) : Tile(x, y) {
	
	//update current name to have a PE at the front of the name
	this->SetName("pe-" + this->GetName());
	
	//create a cpu and memory in addition to current tile hardware
	_mem0   = new UMemory(this->GetName() + ".mem0", MEM0_SIZE, MEM0_BASE); //main
	_cpu    = new THellfireProcessor(this->GetName() + ".cpu", this->GetCommIntr());
	
	//update naming of internal hardware parts (from internal class)
	this->GetRouter()->SetName(this->GetName() + ".router");
	this->GetNetif()->SetName(this->GetName() + ".netif");
	this->GetMem1()->SetName(this->GetName() + ".mem1");
	this->GetMem2()->SetName(this->GetName() + ".mem2");

	//bind control signals to hardware (cpu side)
	this->GetCommAck()->MapTo(_mem0->GetMap(COMM_NOC_ACK), COMM_NOC_ACK);
	this->GetCommStart()->MapTo(_mem0->GetMap(COMM_NOC_START), COMM_NOC_START);
	this->GetCommStatus()->MapTo(_mem0->GetMap(COMM_NOC_STATUS), COMM_NOC_STATUS);
		
	//bind self-id wire (care to save the value before the bind)
	this->GetCommId()->MapTo((uint32_t*)(_mem0->GetMap(COMM_ID)), COMM_ID);
	
	//bind hosttime wire
	this->GetCommHostTime()->MapTo((uint32_t*)(_mem0->GetMap(COMM_HOSTTIME)), COMM_HOSTTIME);
		
	//bind memory modules
	_cpu->SetMem0(_mem0);
	_cpu->SetMem1(this->GetMem1());
	_cpu->SetMem2(this->GetMem2());
		
	//initialize counters for memory modules
	//NOTE: mem0 is initialized here, mem1 and mem2
	//are initialized in Tile.cpp (due inheritance)
	#ifdef MEMORY_ENABLE_COUNTERS
	_mem0->InitCounters(MEM0_COUNTERS_STORE_ADDR, MEM0_COUNTERS_LOAD_ADDR);
	#endif
	
	//initialize counters for the cpu
	#ifdef HFRISCV_ENABLE_COUNTERS
	_cpu->InitCounters(
		CPU_COUNTERS_IARITH_ADDR,
		CPU_COUNTERS_ILOGICAL_ADDR,
		CPU_COUNTERS_ISHIFT_ADDR,
		CPU_COUNTERS_IBRANCHES_ADDR,
		CPU_COUNTERS_IJUMPS_ADDR,
		CPU_COUNTERS_ILOADSTORE_ADDR
	);
	#endif
}

ProcessingTile::~ProcessingTile(){

	delete(_cpu);
	delete(_mem0);	
}


THellfireProcessor* ProcessingTile::GetCpu(){
	return _cpu;
} 

UMemory* ProcessingTile::GetMem0(){
	return _mem0;
}

std::string ProcessingTile::ToString(){
	stringstream ss;
	ss << this->GetName() << "={" << _cpu->GetName() 
	   << ", " << this->GetRouter()->GetName() 
	   << ", " << this->GetNetif()->GetName() << "}";
	
	return ss.str();
}