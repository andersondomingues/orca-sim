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
#include <TDmaNetif.h>
#include <TRouter.h>
#include <UMemory.h>
#include <USignal.h>

#define MEM1_SIZE 0x00000080 /* recv memory */
#define MEM1_BASE 0x50000000

#define MEM2_SIZE 0x00000080 /* send memory */
#define MEM2_BASE 0x50000080

//->>>> first available address for memory mapping 0x40410000

//0x40410000 => memory mapped control wires
#define SIGNAL_CPU_STALL    0x40410000  /* 8 bits */
#define SIGNAL_CPU_INTR     0x40410001
#define SIGNAL_SEND_STATUS  0x40410002

#define SIGNAL_RECV_STATUS  0x40410004 


#define SIGNAL_PROG_SEND    0x40410008
#define SIGNAL_PROG_RECV    0x40410009

#define SIGNAL_PROG_ADDR    0x4041000C  /* 32 bits */
#define SIGNAL_PROG_SIZE    0x40410010

#define MAGIC_TILE_ID       0x40411000  

//0x403F1xxx => memory mapped counters
#ifdef MEMORY_ENABLE_COUNTERS
#define M0_COUNTER_STORE_ADDR (0x40411010)
#define M0_COUNTER_LOAD_ADDR  (0x40411014)
#define M1_COUNTER_STORE_ADDR (0x40411018)
#define M1_COUNTER_LOAD_ADDR  (0x4041101C)
#define M2_COUNTER_STORE_ADDR (0x40411020)
#define M2_COUNTER_LOAD_ADDR  (0x40411024)
#endif

#ifdef HFRISCV_ENABLE_COUNTERS
#define CPU_COUNTER_ARITH_ADDR     (0x40411128)
#define CPU_COUNTER_LOGICAL_ADDR   (0x4041112C)
#define CPU_COUNTER_SHIFT_ADDR     (0x40411130)
#define CPU_COUNTER_BRANCHES_ADDR  (0x40411134)
#define CPU_COUNTER_JUMPS_ADDR     (0x40411138)
#define CPU_COUNTER_LOADSTORE_ADDR (0x4041113C)
#define CPU_COUNTER_HOSTTIME_ADDR  (0x40411140)
#define CPU_COUNTER_CYCLES_TOTAL_ADDR (0x40411144)
#define CPU_COUNTER_CYCLES_STALL_ADDR (0x40411148)
#endif

//0x403F15xx => router wires
#ifdef ROUTER_ENABLE_COUNTERS
#define ROUTER_COUNTER_ACTIVE_ADDR (0x40411500)
#endif

class Tile{

private:

	std::string _name;

	TDmaNetif* _netif;  //network interface 
	TRouter*   _router; //hermes router
	
	UMemory* _mem1; //recv memory
	UMemory* _mem2; //send memory
		
	USignal<int8_t>* _signal_stall;
	USignal<int8_t>* _signal_intr;
	
	USignal<int8_t>* _signal_send_status;
	USignal<int32_t>* _signal_recv_status;
	
	USignal<int8_t>* _signal_prog_send;
	USignal<int8_t>* _signal_prog_recv;
	
	USignal<int32_t>* _signal_prog_addr;
	USignal<int32_t>* _signal_prog_size;	
	
	//self-id wire
	USignal<uint32_t>* _signal_id;

public: 

	/*** ctor. & dtor. ***/
	Tile(uint32_t x, uint32_t y);
	~Tile();
	
	/*** getters ***/
	TRouter*   GetRouter();
	TDmaNetif* GetDmaNetif();

	UMemory* GetMem1();
	UMemory* GetMem2();
	
	//getters
    USignal<int8_t>*  GetSignalStall();
	USignal<int8_t>*  GetSignalIntr();
	USignal<int8_t>*  GetSignalSendStatus();
	USignal<int32_t>* GetSignalRecvStatus();
	USignal<int32_t>* GetSignalProgAddr();
	USignal<int32_t>* GetSignalProgSize();
	USignal<int8_t>*  GetSignalProgSend();
	USignal<int8_t>*  GetSignalProgRecv();

	//setters
    void SetSignalStall(USignal<int8_t>*);
	void SetSignalIntr(USignal<int8_t>*);
	void SetSignalSendStatus(USignal<int8_t>*);
	void SetSignalRecvStatus(USignal<int32_t>*);
	void SetSignalProgAddr(USignal<int32_t>*);
	void SetSignalProgSize(USignal<int32_t>*);
	void SetSignalProgSend(USignal<int8_t>*);
	void SetSignalProgRecv(USignal<int8_t>*);
	
	USignal<uint32_t>* GetSignalId();
	
	std::string GetName();
	
	/*** setters ***/
	void SetName(std::string);
};


#endif /* TILE_H */
