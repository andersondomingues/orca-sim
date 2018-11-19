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
#include <sstream>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>

#include <TNetSocket.h>

/**
 * @brief Default ctor. Name of the module must be informed.
 * @param name A name to identify this module.
 */
TNetSocket::TNetSocket(std::string name) : TimedModel(name) {
	std::cout << this->GetName() << " (warning): this module is unable to run in Windows machines." << std::endl;
	this->Reset();
}

/**
 * @brief Dtor. No dynamic allocation is being used. Keept by design.
 */
TNetSocket::~TNetSocket(){}

/**
 * @brief Return the module to its initial state (if stateful).
 */
void TNetSocket::Reset(){}

/**
 * @brief Runs a state.
 * @return The number of cycles spent to change (or not) states.
 */
long long unsigned int TNetif::Run(){
    
	this->recvProcess(); //process for receiving from the UDP socket
    this->sendProcess(); //process for sending through the UDP socket
    return 1; //takes exactly 1 cycle to run both processes
}

void TNetSocket::recvProcess(){
	
	
	
}


void TNetSocket::sendProcess(){	

	
}
