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
#include <TNetif.h>
#include <TRouter.h>
#include <UMemory.h>
#include <TNetSocket.h>

#include <Tile.h>

/** 
 * Default constructor.
 * Instantiate and bind internal hardware to each
 * other. */
Tile::Tile(uint32_t x, uint32_t y){
	
	//create new memories	
	_mem1   = new UMemory(_name + ".mem1", MEM1_SIZE, MEM1_BASE); //read from noc 
	_mem2   = new UMemory(_name + ".mem2", MEM2_SIZE, MEM2_BASE); //write to noc

	//peripherals	
	_router = new TRouter(_name + ".router", x, y);
	_netif  = new TNetif(_name + ".netif");

	//control signals to receive packets from netif
	_socket_ack  = new UComm<int8_t>("socket_ack",  0, COMM_NOC_ACK);
	_socket_intr = new UComm<int8_t>("socket_intr", 0, COMM_NOC_INTR);
	
	//control signals to send packets to the netif
	_socket_start  = new UComm<int8_t>("socket_start", 0, COMM_NOC_START);

	//bind control signals to hardware (netif side)
	_netif->SetCommAck(_socket_ack);
	_netif->SetCommIntr(_socket_intr);
	_netif->SetCommStart(_socket_start);
	
	//bind netif to router
	_router->SetOutputBuffer(_netif->GetInputBuffer(), LOCAL);
	_netif->SetOutputBuffer(_router->GetInputBuffer(LOCAL));
	
	//bind memories
	_netif->SetMem1(_mem1);
	_netif->SetMem2(_mem2);
}

Tile::~Tile(){
	
	delete(_router);
	
	delete(_netif);
	
	delete(_mem1);
	delete(_mem2);
	
	delete(_socket_ack);
	delete(_socket_intr);
	delete(_socket_start);
}

/* getters*/
TRouter* Tile::GetRouter(){ return _router; }
TNetif*  Tile::GetNetif(){  return _netif; }
UMemory* Tile::GetMem1(){   return _mem1;}
UMemory* Tile::GetMem2(){ 	return _mem2;}

UComm<int8_t>* Tile::GetCommAck(){ return _socket_ack; }
UComm<int8_t>* Tile::GetCommIntr(){ return _socket_intr; }
UComm<int8_t>* Tile::GetCommStart(){ return _socket_start; }

std::string Tile::GetName(){
	return _name;
}