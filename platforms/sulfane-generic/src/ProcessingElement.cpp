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

//std API
#include <iostream>

#include <TProcessingElement.h>

//model API
#include <THellfireProcessor.h>
#include <TDma.h>
#include <TRouter.h>
#include <UMemory.h>

THellfireProcessor* _cpu;
TDma*    _dma; 
TRouter* _router; //hermes router

UMemory* _mem0; //main memory
UMemory* _mem1; //recv memory
UMemory* _mem2; //send memory

UComm* _cpudma_ack, _cpudma_intr;

TProcessingElement::TProcessingElement(uint32_t x, uint32_t y){
	
	_name = "?";
	
	//create PE hardware
	_mem0   = new UMemory(); //main
	_mem1   = new UMemory(); //read from noc 
	_mem2   = new UMemory(); //write to noc
	
	_router = new TRouter("", x, y);
	_cpu    = new THellfireProcessor("", nullptr, 0, 0);
	_dma    = new TDma();

	//create control signals
	_cpudma_ack  = new UComm("cpudma_ack",  bool);
	_cpudma_intr = new UComm("cpudma_intr", bool);
	
	//bind control signals to hardware
	
	
}

TProcessingElement::~TProcessingElement(){
	delete(_router);
	delete(_cpu);
	delete(_dma);
	delete(_mem0);
	delete(_mem1);
	delete(_mem2);
	delete(_cpudma_ack);
	delete(_cpudma_intr);
}

/* getters*/
TProcessingElement::GetRouter(){
	return _router;
}

TProcessingElement::GetDma(){
	return _dma;
}

TProcessingElement::GetCpu(){
	return _cpu;
} 