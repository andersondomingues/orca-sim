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

#include <ProcessingElement.h>

/** 
 * Default constructor.
 * Instantiate and bind internal hardware to each
 * other. */
ProcessingElement::ProcessingElement(uint32_t x, uint32_t y){
	
	_name = "pe-" + std::to_string(x) + "-" + std::to_string(y);
	
	//create PE hardware
	_mem0   = new UMemory(_name + ".mem0", MEM0_SIZE, MEM0_BASE); //main
	_mem1   = new UMemory(_name + ".mem1", MEM1_SIZE, MEM1_BASE); //read from noc 
	_mem2   = new UMemory(_name + ".mem2", MEM2_SIZE, MEM2_BASE); //write to noc
	
	_router = new TRouter(_name + ".router", x, y);
	_cpu    = new THellfireProcessor(_name + ".cpu");
	_netif  = new TNetif(_name + ".netif");

	//control signals to receive packets from netif
	_cpudma_ack  = new UComm<bool>("cpunetif_ack",  false);
	_cpudma_intr = new UComm<bool>("cpunetif_intr", false);
	
	//control signals to send packets to the netif
	_cpudma_start  = new UComm<bool>("cpunetif_start", false);
	_cpudma_status = new UComm<bool>("cpunetif_status", false);
		
	//bind control signals to hardware (netif side)
	_netif->SetCommAck(_cpudma_ack);
	_netif->SetCommIntr(_cpudma_intr);
	_netif->SetCommStart(_cpudma_start);
	_netif->SetCommStatus(_cpudma_status);
	
	//bind control signals to hardware (cpu side)
	_cpu->SetCommAck(_cpudma_ack);
	_cpu->SetCommIntr(_cpudma_intr);
	_cpu->SetCommStart(_cpudma_start);
	_cpu->SetCommStatus(_cpudma_status);
	
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

ProcessingElement::~ProcessingElement(){
	delete(_router);
	delete(_cpu);
	delete(_netif);
	delete(_mem0);
	delete(_mem1);
	delete(_mem2);
	delete(_cpudma_ack);
	delete(_cpudma_intr);
}

/* getters*/
TRouter* ProcessingElement::GetRouter(){
	return _router;
}

TNetif* ProcessingElement::GetNetif(){
	return _netif;
}

THellfireProcessor* ProcessingElement::GetCpu(){
	return _cpu;
} 

UMemory* ProcessingElement::GetmMem0(){
	return _mem0;
}

UMemory* ProcessingElement::GetmMem1(){
	return _mem1;
}

UMemory* ProcessingElement::GetmMem2(){
	return _mem2;
}

std::string ProcessingElement::GetName(){
	return _name;
}

std::string ProcessingElement::ToString(){
	stringstream ss;
	ss << _name << "{" << std::endl;
	ss << "    " << _cpu->GetName() << std::endl;
	ss << "}" << std::endl;
	
	return ss.str();
}