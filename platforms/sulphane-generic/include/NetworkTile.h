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
#include <TNetSocket.h>
#include <TNetif.h>
#include <TRouter.h>
#include <UMemory.h>
#include <UComm.h>

//netif mem mapping
#define MEM1_SIZE 0x00000080 /* recv memory */
#define MEM1_BASE 0x90000000

#define MEM2_SIZE 0x00000080 /* send memory */
#define MEM2_BASE 0x90000080

//comms
#define COMM_NOC_ACK    0x80000001
#define COMM_NOC_INTR   0x80000002
#define COMM_NOC_START  0x80000003



class NetworkTile{

private:

	std::string _name;

	TNetSocket* _socket; //hfrisv-core
	TNetif*  _netif;  //network interface 
	TRouter* _router; //hermes router
	
	UMemory* _mem1; //recv memory
	UMemory* _mem2; //send memory
	
	
	//recv signals 
	UComm<int8_t>* _socket_ack;
	UComm<int8_t>* _socket_intr;      
	
	//send signals
	UComm<int8_t>* _socket_start;
	
public: 

	NetworkTile(uint32_t x, uint32_t y);
	~NetworkTile();
	
	//getters
	TRouter* GetRouter();
	TNetif*  GetNetif();
	TNetSocket* GetSocket();

	UMemory* GetMem1();
	UMemory* GetMem2();
	
	//comms
	void SetCommAck(UComm<int8_t>*);
	void SetCommIntr(UComm<int8_t>*);
	void SetCommStart(UComm<int8_t>*);
	
	//getters for mems
	UMemory* GetmMem1();
	UMemory* GetmMem2();
	
	std::string GetName();
	std::string ToString();
};


#endif /* TROUTER_H */
