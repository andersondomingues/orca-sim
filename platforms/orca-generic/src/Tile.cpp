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
 * @brief Initialize internal entities and binds hardware wires
 * @param x X-coordinate of the tile in the NoC
 * @param y Y-coordinate of the tile in the NoC
 **/
Tile::Tile(uint32_t x, uint32_t y){
	
	//bind comm id (W * column + line)
	_sid = (ORCA_NOC_WIDTH * y) + x;
	_comm_id = new UComm<uint32_t>(&_sid, COMM_ID, ".id");
	
	//create new memories	
	_mem1   = new UMemory(this->GetName() + ".mem1", MEM1_SIZE, MEM1_BASE); //read from noc 
	_mem2   = new UMemory(this->GetName() + ".mem2", MEM2_SIZE, MEM2_BASE); //write to noc

	//peripherals	
	_router = new TRouter(this->GetName() + ".router", x, y);
	_netif  = new TNetif (this->GetName() + ".netif");

	//ni wires
	_comm_ack    = new UComm<int8_t>(&_sack,   COMM_NOC_ACK,   this->GetName() + ".ack");
	_comm_intr   = new UComm<int8_t>(&_sintr,  COMM_NOC_INTR,  this->GetName() + ".intr");
	_comm_start  = new UComm<int8_t>(&_sstart, COMM_NOC_START, this->GetName() + ".start");
	_comm_status = new UComm<int8_t>(&_sstatus,COMM_NOC_STATUS,this->GetName() + ".status");
	
	//systime
	_comm_hosttime = new UComm<uint32_t>(&_shosttime, COMM_SYSTIME, this->GetName() + ".systime");
	
	//bind control signals to hardware (netif side)
	_netif->SetCommAck   (_comm_ack);
	_netif->SetCommIntr  (_comm_intr);
	_netif->SetCommStart (_comm_start);
	_netif->SetCommStatus(_comm_status);
	
	//bind netif to router
	_router->SetOutputBuffer(_netif->GetInputBuffer(), LOCAL);
	_netif->SetOutputBuffer(_router->GetInputBuffer(LOCAL));
	
	//bind memories
	_netif->SetMem1(_mem1);
	_netif->SetMem2(_mem2);
		
	_name = "tile" + std::to_string(_sid);
			
	//counter initialization
	#ifdef MEMORY_ENABLE_COUNTERS
	_mem1->InitCounters(MEM1_COUNTERS_STORE_ADDR, MEM1_COUNTERS_LOAD_ADDR);
	_mem2->InitCounters(MEM2_COUNTERS_STORE_ADDR, MEM2_COUNTERS_LOAD_ADDR);
	#endif
	
	#ifdef ROUTER_ENABLE_COUNTERS
	_router->InitCounters(ROUTER_COUNTERS_ACTIVE_ADDR);
	#endif
}

/**
 * @brief Destructor: cleans up allocated memory 
 */
Tile::~Tile(){
	
	//delete hardware modules
	delete(_router);
	delete(_netif);
	delete(_mem1);
	delete(_mem2);
	
	//delete comms 
	delete(_comm_ack);
	delete(_comm_intr);
	delete(_comm_start);
	delete(_comm_status);
}

/************************************* GETTERS **************************************/
/**
 * @brief Get current router of the PE
 * @return A pointer to the instance of router
 */
TRouter* Tile::GetRouter(){ 
	return _router; 
}

/**
 * @brief Get current NI module
 * @return A pointer to the instance of NI
 */
TNetif*  Tile::GetNetif(){
	return _netif;
}

/**
 * @brief Get sender memory module 
 * @return A pointer to the instance of memory
 */
UMemory* Tile::GetMem1(){
	return _mem1;
}

/**
 * @brief Get recv memory module
 * @return A pointer to the instance of memory
 */
UMemory* Tile::GetMem2(){
	return _mem2;
}

/**
 * @brief Get current tile name
 * @return The name of the tile
 */
std::string Tile::GetName(){
	return _name;
}

/**
 * @brief Get current comm for tile ID
 * @return A pointer to the instance of comm
 */
UComm<uint32_t>* Tile::GetCommId(){ 
	return _comm_id; 
}

/**
 * @brief Get current comm for ack signal
 * @return A pointer to the instance of comm
 */
UComm<int8_t>* Tile::GetCommAck(){
	return _comm_ack; 
}

/**
 * @brief Get current comm for intr signal
 * @return A pointer to the instance of comm
 */
UComm<int8_t>* Tile::GetCommIntr(){
	return _comm_intr;
}

/**
 * @brief Get current comm for start signal
 * @return A pointer to the instance of comm
 */
UComm<int8_t>* Tile::GetCommStart(){
	return _comm_start;
}

/**
 * @brief Get current comm for systime signal
 * @return A pointer to the instance of comm
 */
UComm<uint32_t>* Tile::GetCommHostTime(){
	return _comm_hosttime;
}

/************************************* SETTERS **************************************/
/**
 * @brief Set a name to this tile
 * @param name Name to be given to the tile. Please note that the name is autogenerated
 * in accordance to the structure of the tile, so updating the name of the tile may 
 * not reflect on the naming of internal structures (to fix)
 */
void Tile::SetName(std::string name){
	_name = name;
}