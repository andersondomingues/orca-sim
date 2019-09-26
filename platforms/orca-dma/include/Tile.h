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

/** Memory range from "403Fxxxx" to "40FFxxxx" is reserved
  * for control signals and magic signals. These signals are
  * required by the platform to work properly */
#define SIGNAL_CPU_STALL    0x403F0000  /* 8 bits */
#define SIGNAL_CPU_INTR     0x403F0001
#define SIGNAL_SEND_STATUS  0x403F0002

#define SIGNAL_RECV_STATUS  0x403F0004 

#define SIGNAL_PROG_SEND    0x403F0008
#define SIGNAL_PROG_RECV    0x403F0009
//xxF0006 and xxF0007 skipped to keep alignment

#define SIGNAL_PROG_ADDR    0x403F000C  /* 32 bits */
#define SIGNAL_PROG_SIZE    0x403F0010

#define MAGIC_TILE_ID       0x403F1000  
#define MAGIC_HOSTTIME      0x403F1004

/** Memory range from "81xxxxxx" to "81FFxxxx" is reserved 
  * for internal counters and user-defined magic signals.
  * These signals are not required by the platform, although
  * they can be required by one service or another. */
#ifdef MEMORY_ENABLE_COUNTERS
#define MEM0_COUNTERS_STORE_ADDR 0x81000000
#define MEM0_COUNTERS_LOAD_ADDR  0x81000004
#define MEM1_COUNTERS_STORE_ADDR 0x81000008
#define MEM1_COUNTERS_LOAD_ADDR  0x8100000C
#define MEM2_COUNTERS_STORE_ADDR 0x81000010
#define MEM2_COUNTERS_LOAD_ADDR  0x81000014
#endif

#ifdef HFRISCV_ENABLE_COUNTERS
#define CPU_COUNTERS_IARITH_ADDR     0x81000100 
#define CPU_COUNTERS_ILOGICAL_ADDR   0x81000104
#define CPU_COUNTERS_ISHIFT_ADDR     0x81000108
#define CPU_COUNTERS_IBRANCHES_ADDR  0x8100010C
#define CPU_COUNTERS_IJUMPS_ADDR     0x81000110
#define CPU_COUNTERS_ILOADSTORE_ADDR 0x81000114
#endif

#ifdef ROUTER_ENABLE_COUNTERS
#define ROUTER_COUNTERS_ACTIVE_ADDR 0x81000200
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
	
	//hosttime magic wire
	uint32_t _shosttime;
	USignal<uint32_t>* _signal_hosttime;

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
	USignal<int32_t>*  GetSignalRecvStatus();
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
	USignal<uint32_t>* GetSignalHostTime();

	UMemory* GetmMem1();
	UMemory* GetmMem2();
	
	std::string GetName();
	
	/*** setters ***/
	void SetName(std::string);
};


#endif /* TILE_H */
