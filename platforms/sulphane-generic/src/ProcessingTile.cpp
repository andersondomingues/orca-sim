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
ProcessingTile::ProcessingTile(uint32_t x, uint32_t y){
	
	_name = "pe-" + std::to_string(x) + "-" + std::to_string(y);
	
	_mem1   = new UMemory(_name + ".mem1", MEM1_SIZE, MEM1_BASE); //read from noc 
	_mem2   = new UMemory(_name + ".mem2", MEM2_SIZE, MEM2_BASE); //write to noc
	
	//create PE hardware
	_mem0   = new UMemory(_name + ".mem0", MEM0_SIZE, MEM0_BASE); //main
	
	_router = new TRouter(_name + ".router", x, y);
	_cpu    = new THellfireProcessor(_name + ".cpu");
	_netif  = new TNetif(_name + ".netif");

	//control signals to receive packets from netif
	_cpudma_ack  = new UComm<int8_t>("cpunetif_ack",  0, COMM_NOC_ACK);
	_cpudma_intr = new UComm<int8_t>("cpunetif_intr", 0, COMM_NOC_INTR);
	
	//control signals to send packets to the netif
	_cpudma_start  = new UComm<int8_t>("cpunetif_start", 0, COMM_NOC_START);

	//bind control signals to hardware (netif side)
	_netif->SetCommAck(_cpudma_ack);
	_netif->SetCommIntr(_cpudma_intr);
	_netif->SetCommStart(_cpudma_start);
	
	//bind control signals to hardware (cpu side)
	_cpu->SetCommAck(_cpudma_ack);
	_cpu->SetCommIntr(_cpudma_intr);
	_cpu->SetCommStart(_cpudma_start);
	
	//bind netif to router
	_router->SetOutputBuffer(_netif->GetInputBuffer(), LOCAL);
	_netif->SetOutputBuffer(_router->GetInputBuffer(LOCAL));
	
	//bind memory modules
	_cpu->SetMem0(_mem0);
	_cpu->SetMem1(_mem1);
	_cpu->SetMem2(_mem2);
	
	_netif->SetMem1(_mem1);
	_netif->SetMem2(_mem2);
}

ProcessingTile::~ProcessingTile(){
	
	delete(_router);
	delete(_cpu);
	delete(_netif);
	
	delete(_mem0);
	delete(_mem1);
	delete(_mem2);
	
	delete(_cpudma_ack);
	delete(_cpudma_intr);
	delete(_cpudma_start);
}

/* getters*/
TRouter* ProcessingTile::GetRouter(){
	return _router;
}

TNetif* ProcessingTile::GetNetif(){
	return _netif;
}

THellfireProcessor* ProcessingTile::GetCpu(){
	return _cpu;
} 

UMemory* ProcessingTile::GetMem0(){
	return _mem0;
}

UMemory* ProcessingTile::GetMem1(){
	return _mem1;
}

UMemory* ProcessingTile::GetMem2(){
	return _mem2;
}

std::string ProcessingTile::GetName(){
	return _name;
}

std::string ProcessingTile::ToString(){
	stringstream ss;
	ss << _name << "={" << _cpu->GetName() 
	   << ", " << _router->GetName() 
	   << ", " << _netif->GetName() << "}";
	
	return ss.str();
}