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
NetworkTile::NetworkTile(uint32_t x, uint32_t y) : Tile(x, y){
	
	this->SetName("nt-" + this->GetName());
	
	//peripherals	
	_socket = new TNetSocket(_name + ".sock");
	
	//bind control signals to hardware (socket side)
	_socket->SetCommAck(this->GetCommAck());
	_socket->SetCommIntr(this->GetCommIntr());
	_socket->SetCommStart(this->GetCommStart());
	
	//bind memory modules
	_socket->SetMem1(this->GetMem1()); //recv
	_socket->SetMem2(this->GetMem2()); //send
}

NetworkTile::~NetworkTile(){
	
	delete(_socket);
}

TNetSocket* NetworkTile::GetSocket(){
	return _socket;
} 

std::string NetworkTile::ToString(){
	stringstream ss;
	ss << _name << "={" << _socket->GetName() 
	   << ", " << this->GetRouter()->GetName() 
	   << ", " << this->GetNetif()->GetName() << "}";
	
	return ss.str();
}