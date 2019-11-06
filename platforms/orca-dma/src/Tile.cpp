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
#include <iomanip>

//model API
#include <TDmaNetif.h>
#include <TRouter.h>
#include <UMemory.h>

#include <Tile.h>

/** 
 * @brief Initialize internal entities and binds hardware wires
 * @param x X-coordinate of the tile in the NoC
 * @param y Y-coordinate of the tile in the NoC
 **/
Tile::Tile(uint32_t x, uint32_t y){
	
	//map id wire to memory (local storage)
	uint32_t id = (ORCA_NOC_WIDTH * y) + x;
	
	stringstream ss;
	ss << std::setw(3) << std::setfill('0') << id;
	
	_name = ss.str();
	
	_signal_id = new USignal<uint32_t>(MAGIC_TILE_ID, this->GetName() + ".id");
	_signal_id->Write(id);
	
	//peripherals	
	_router = new TRouter(this->GetName() + ".router", x, y);
	_netif  = new TDmaNetif (this->GetName() + ".netif");

	//ni sig wires
	_signal_stall       = new USignal<int8_t>(SIGNAL_CPU_STALL, this->GetName() + ".stall");
	_signal_intr        = new USignal<int8_t>(SIGNAL_CPU_INTR,  this->GetName() + ".intr");
	_signal_send_status = new USignal<int8_t>(SIGNAL_SEND_STATUS, this->GetName() + ".send_status");
	_signal_recv_status = new USignal<int32_t>(SIGNAL_RECV_STATUS, this->GetName() + ".recv_status");
	_signal_prog_send   = new USignal<int8_t>(SIGNAL_PROG_SEND, this->GetName() + ".progr_send");
	_signal_prog_recv   = new USignal<int8_t>(SIGNAL_PROG_RECV, this->GetName() + ".progr_recv");
	_signal_prog_addr   = new USignal<int32_t>(SIGNAL_PROG_ADDR, this->GetName() + ".progr_addr");
	_signal_prog_size   = new USignal<int32_t>(SIGNAL_PROG_SIZE, this->GetName() + ".progr_size");

	//reset control wires
    _signal_stall->Write(0);
	_signal_intr->Write(0); 
	_signal_send_status->Write(0);
	_signal_recv_status->Write(0);
	_signal_prog_send->Write(0);
	_signal_prog_recv->Write(0);
	_signal_prog_addr->Write(0);
	_signal_prog_size->Write(0);
		
	//bind control signals to hardware (netif side)
	_netif->SetSignalStall(_signal_stall);
	_netif->SetSignalIntr(_signal_intr);
	_netif->SetSignalSendStatus(_signal_send_status);
	_netif->SetSignalRecvStatus(_signal_recv_status);
	_netif->SetSignalProgSend(_signal_prog_send);
	_netif->SetSignalProgRecv(_signal_prog_recv);
	_netif->SetSignalProgAddr(_signal_prog_addr);
	_netif->SetSignalProgSize(_signal_prog_size);
	
	//bind netif to router
	_router->SetOutputBuffer(_netif->GetInputBuffer(), LOCAL);
	_netif->SetOutputBuffer(_router->GetInputBuffer(LOCAL));

	//create new memories for the NI
	_mem1 = new UMemory(this->GetName() + ".mem1", MEM1_SIZE, 0); //read from noc 
	_mem2 = new UMemory(this->GetName() + ".mem2", MEM2_SIZE, 0); //write to noc

	//bind memories
	_netif->SetMem1(_mem1);	
	_netif->SetMem2(_mem2);

	//counter initialization
	#ifdef MEMORY_ENABLE_COUNTERS
	_mem1->InitCounters(M1_COUNTER_STORE_ADDR, M1_COUNTER_LOAD_ADDR);
	_mem2->InitCounters(M2_COUNTER_STORE_ADDR, M2_COUNTER_LOAD_ADDR);
	#endif
	
	#ifdef ROUTER_ENABLE_COUNTERS
	_router->InitCounters(ROUTER_COUNTER_ACTIVE_ADDR);
	#endif
}

/**
 * @brief Destructor: cleans up allocated memory 
 */
Tile::~Tile(){
	
	delete(_signal_id);
	
	//delete hardware modules
	delete(_router);
	delete(_netif);
	delete(_mem1);
	delete(_mem2);
	
	//delete signals 
	delete(_signal_stall);
	delete(_signal_intr);
	delete(_signal_send_status);
	delete(_signal_recv_status);
	delete(_signal_prog_send);
	delete(_signal_prog_recv);
	delete(_signal_prog_addr);
	delete(_signal_prog_size);	
}

/************************************* GETTERS **************************************/
USignal<int8_t>*  Tile::GetSignalStall(){ return _signal_stall; }
USignal<int8_t>*  Tile::GetSignalIntr(){ return _signal_intr; }
USignal<int8_t>*  Tile::GetSignalSendStatus(){ return _signal_send_status; }
USignal<int32_t>*  Tile::GetSignalRecvStatus(){ return _signal_recv_status; }
USignal<int32_t>* Tile::GetSignalProgAddr(){ return _signal_prog_addr; }
USignal<int32_t>* Tile::GetSignalProgSize(){ return _signal_prog_size; }
USignal<int8_t>*  Tile::GetSignalProgSend(){ return _signal_prog_send; }
USignal<int8_t>*  Tile::GetSignalProgRecv(){ return _signal_prog_recv; }


/*
	//setters
    void SetSignalStall(USignal<int8_t>*);
	void SetSignalIntr(USignal<int8_t>*);
	void SetSignalSendStatus(USignal<int8_t>*);
	void SetSignalRecvStatus(USignal<int8_t>*);
	void SetSignalProgAddr(USignal<int32_t>*);
	void SetSignalProgSize(USignal<int32_t>*);
	void SetSignalProgSend(USignal<int8_t>*);
	void SetSignalProgRecv(USignal<int8_t>*);
*/

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
TDmaNetif*  Tile::GetDmaNetif(){
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