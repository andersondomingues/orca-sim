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
#ifndef __TPROCESSINGELEMENT_H
#define __TPROCESSINGELEMENT_H

//std API
#include <iostream>

//model API
#include <THellfireProcessor.h>
#include <TNetif.h>
#include <TRouter.h>
#include <UMemory.h>


/**
 * @class TProcessingElement
 * @author Anderson Domingues
 * @date 10/04/18
 * @file TProcessingElement.h
 * @brief This class models an entire processing element that contains
 * RAM memory (3x), DMA, NoC Router, HFRiscV core and an SPI interface. 
 */
class TProcessingElement{

private:

	THellfireProcessor* _cpu; //hfrisv-core
	TNetif*  _netif;  //network interface 
	TRouter* _router; //hermes router
	
	UMemory* _mem0; //main memory
	UMemory* _mem1; //recv memory
	UMemory* _mem2; //send memory
	
	UComm* _cpudma_ack, _cpudma_intr;      //recv signals 
	UComm* _cpudma_start, _cpudma_status;  //send signals
	
public: 

	TProcessingElement(uint32_t x, uint32_t y);
	~TProcessingElement();
	
	//getters
	TRouter* GetRouter();
	TNetif*  GetNetif();
	THellfireProcessor* GetCpu();
	UMemory* GetMem0();
	UMemory* GetMem1();
	UMemory* GetMem2();
	
	std::string GetName();
};


#endif /* TROUTER_H */
