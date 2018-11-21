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

#include <NetworkTile.h>

/** 
 * Default constructor.
 * Instantiate and bind internal hardware to each
 * other. */
NetworkTile::NetworkTile(uint32_t x, uint32_t y){
	
	_name = "pe-" + std::to_string(x) + "-" + std::to_string(y);

	//create new memories	
	_mem1   = new UMemory(_name + ".mem1", MEM1_SIZE, MEM1_BASE); //read from noc 
	_mem2   = new UMemory(_name + ".mem2", MEM2_SIZE, MEM2_BASE); //write to noc

	//peripherals	
	_router = new TRouter(_name + ".router", x, y);
	_socket    = new TNetSocket(_name + ".sock");
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
	
	//bind control signals to hardware (socket side)
	_socket->SetCommAck(_socket_ack);
	_socket->SetCommIntr(_socket_intr);
	_socket->SetCommStart(_socket_start);
	
	//bind netif to router
	_router->SetOutputBuffer(_netif->GetInputBuffer(), LOCAL);
	_netif->SetOutputBuffer(_router->GetInputBuffer(LOCAL));
	
	//bind memory modules
	_socket->SetMem1(_mem1); //recv
	_socket->SetMem2(_mem2); //send
	
	_netif->SetMem1(_mem1);
	_netif->SetMem2(_mem2);
}

NetworkTile::~NetworkTile(){
	
	delete(_router);
	delete(_socket);
	delete(_netif);
	
	delete(_mem1);
	delete(_mem2);
	
	delete(_socket_ack);
	delete(_socket_intr);
	delete(_socket_start);
}

/* getters*/
TRouter* NetworkTile::GetRouter(){
	return _router;
}

TNetif* NetworkTile::GetNetif(){
	return _netif;
}

TNetSocket* NetworkTile::GetSocket(){
	return _socket;
} 

UMemory* NetworkTile::GetMem1(){
	return _mem1;
}

UMemory* NetworkTile::GetMem2(){
	return _mem2;
}

std::string NetworkTile::GetName(){
	return _name;
}

std::string NetworkTile::ToString(){
	stringstream ss;
	ss << _name << "={" << _socket->GetName() 
	   << ", " << _router->GetName() 
	   << ", " << _netif->GetName() << "}";
	
	return ss.str();
}