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
	
	//map id wire to memory (local storage)
	uint32_t id = (ORCA_NOC_WIDTH * y) + x;
	_name = "tile" + std::to_string(id);
	
	_signal_id = new USignal<uint32_t>(COMM_ID, this->GetName() + ".id");
	_signal_id->Write(id);
	
	//hosttime signal
	_signal_hosttime = new USignal<uint32_t>(COMM_HOSTTIME,this->GetName() + ".hosttime");
	//@TODO: bind to function?!
	
	//create new memories	
	_mem1 = new UMemory(this->GetName() + ".mem1", MEM1_SIZE, MEM1_BASE); //read from noc 
	_mem2 = new UMemory(this->GetName() + ".mem2", MEM2_SIZE, MEM2_BASE); //write to noc

	//peripherals	
	_router = new TRouter(this->GetName() + ".router", x, y);
	_netif  = new TNetif (this->GetName() + ".netif");

	//ni wires
	_signal_ack    = new USignal<int8_t>(COMM_NOC_ACK,   this->GetName() + ".ack");
	_signal_intr   = new USignal<int8_t>(COMM_NOC_INTR,  this->GetName() + ".intr");
	_signal_start  = new USignal<int8_t>(COMM_NOC_START, this->GetName() + ".start");
	_signal_status = new USignal<int8_t>(COMM_NOC_STATUS,this->GetName() + ".status");
		
	//reset control wires
	_signal_ack->Write(0);
	_signal_intr->Write(0);
	_signal_start->Write(0);
	_signal_status->Write(0);
		
	//bind control signals to hardware (netif side)
	_netif->SetSignalAck   (_signal_ack);
	_netif->SetSignalIntr  (_signal_intr);
	_netif->SetSignalStart (_signal_start);
	_netif->SetSignalStatus(_signal_status);
	
	//bind netif to router
	_router->SetOutputBuffer(_netif->GetInputBuffer(), LOCAL);
	_netif->SetOutputBuffer(_router->GetInputBuffer(LOCAL));
	
	//bind memories
	_netif->SetMem1(_mem1);
	_netif->SetMem2(_mem2);
				
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
	
	//delete signals 
	delete(_signal_ack);
	delete(_signal_intr);
	delete(_signal_start);
	delete(_signal_status);
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
 * @brief Get current signal for tile ID
 * @return A pointer to the instance of signal
 */
USignal<uint32_t>* Tile::GetSignalId(){ 
	return _signal_id; 
}

/**
 * @brief Get current signal for ack signal
 * @return A pointer to the instance of signal
 */
USignal<int8_t>* Tile::GetSignalAck(){
	return _signal_ack; 
}

/**
 * @brief Get current signal for intr signal
 * @return A pointer to the instance of signal
 */
USignal<int8_t>* Tile::GetSignalIntr(){
	return _signal_intr;
}

/**
 * @brief Get current signal for start signal
 * @return A pointer to the instance of signal
 */
USignal<int8_t>* Tile::GetSignalStart(){
	return _signal_start;
}

/**
 * @brief Get current signal for status signal
 * @return A pointer to the instance of signal
 */
USignal<int8_t>* Tile::GetSignalStatus(){
	return _signal_status;
}

/**
 * @brief Get current signal for systime signal
 * @return A pointer to the instance of signal
 */
USignal<uint32_t>* Tile::GetSignalHostTime(){
	return _signal_hosttime;
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