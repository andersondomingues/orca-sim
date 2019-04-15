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

#ifndef __TILE_H
#define __TILE_H

//std API
#include <iostream>

//model API
#include <TNetif.h>
#include <TRouter.h>
#include <UMemory.h>
#include <UComm.h>

//netif mem mapping
#define MEM1_SIZE 0x00000080 /* recv memory */
#define MEM1_BASE 0x90000000

#define MEM2_SIZE 0x00000080 /* send memory */
#define MEM2_BASE 0x90000080

//comms (80xx.. to 80ff.. reserved for gp wires)
#define COMM_NOC_ACK    0x80000000
#define COMM_NOC_INTR   0x80000001
#define COMM_NOC_START  0x80000002

//self id addr
#define COMM_ID         0x80000010 

//counters (81xx.. to 81ff..) reserved for internal counters
#ifndef OPT_MEMORY_DISABLE_COUNTERS
#define MEM0_COUNTERS_STORE_ADDR 0x81000000
#define MEM0_COUNTERS_LOAD_ADDR  0x81000004
#define MEM1_COUNTERS_STORE_ADDR 0x81000008
#define MEM1_COUNTERS_LOAD_ADDR  0x8100000C
#define MEM2_COUNTERS_STORE_ADDR 0x81000010
#define MEM2_COUNTERS_LOAD_ADDR  0x81000014
#endif

#ifndef OPT_HFRISC_DISABLE_COUNTERS
#define CPU_COUNTERS_IARITH_ADDR     0x81000100 
#define CPU_COUNTERS_ILOGICAL_ADDR   0x81000104
#define CPU_COUNTERS_ISHIFT_ADDR     0x81000108
#define CPU_COUNTERS_IBRANCHES_ADDR  0x8100010C
#define CPU_COUNTERS_IJUMPS_ADDR     0x81000110
#define CPU_COUNTERS_ILOADSTORE_ADDR 0x81000114
#endif

class Tile{

private:

	std::string _name;

	TNetif*  _netif;  //network interface 
	TRouter* _router; //hermes router
	
	UMemory* _mem1; //recv memory
	UMemory* _mem2; //send memory
	
	//recv signals 
	UComm<int8_t>* _socket_ack;
	UComm<int8_t>* _socket_intr;      
	
	//send signals
	UComm<int8_t>* _socket_start;
	
	//self id
	UComm<int32_t>* _comm_id;
	
public: 

	Tile(uint32_t x, uint32_t y);
	~Tile();
	
	//getters
	TRouter* GetRouter();
	TNetif*  GetNetif();

	UMemory* GetMem1();
	UMemory* GetMem2();
	
	//comms
	void SetCommAck(UComm<int8_t>*);
	void SetCommIntr(UComm<int8_t>*);
	void SetCommStart(UComm<int8_t>*);
	
	void SetCommId(UComm<int32_t>*);
	
	UComm<int8_t>* GetCommAck();
	UComm<int8_t>* GetCommIntr();
	UComm<int8_t>* GetCommStart();
	
	UComm<int32_t>* GetCommId();
	
	
	//getters for mems
	UMemory* GetmMem1();
	UMemory* GetmMem2();
	
	std::string GetName();
	void SetName(std::string);
	
	std::string ToString();
};


#endif /* TILE_H */
