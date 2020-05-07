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
#ifndef __ProcessingTile_H
#define __ProcessingTile_H

//std API
#include <iostream>

//model API
#include <THFRiscV.h>
#include <UMemory.h>
#include <USignal.h>

//include petipheral addresses
#include <MemoryMap.h>

/* MEMORY LAYOUT
------------------- 0x40000000 <<-- code begin

       sram
      (~4MB)

------------------- 0x40400000 <<-- stack
     empty space
------------------- 0x40410000 <<-- mmio begin

       mmio
      (~FFFF)

------------------- 0x4041FFFF <<-- mmio end

*/

//memory mapping
//#define MEM0_SIZE 0x0041FFFF /* main memory */
//#define MEM0_BASE 0x40000000

#define MEM0_BASE ORCA_MEMORY_BASE
#define MEM0_SIZE ORCA_MEMORY_SIZE
/**
 * @class ProcessingTile
 * @author Anderson Domingues
 * @date 10/04/18
 * @file ProcessingTile.h
 * @brief This class models an entire processing element that contains
 * RAM memory (3x), DMA, NoC Router, HFRiscV core
 */
class ProcessingTile{

private:

	THFRiscV* _cpu; //hfrisv-core

	//main memory
	UMemory* _mem0;
	
	//NOTE: other hardware is defined in Tile.h as 
	//we use inheritance to derive multiple tiles 
	//with similar architecture.
	
	//hosttime magic wire
	uint32_t _shosttime;
	USignal<uint32_t>* _signal_hosttime;
	
	USignal<uint8_t>*  _signal_stall;
	USignal<uint8_t>*  _signal_intr;
	
public: 

	ProcessingTile();
	~ProcessingTile();
	
	//getters
    	USignal<uint8_t>*  GetSignalStall();
	USignal<uint8_t>*  GetSignalIntr();

	//setters
	void SetSignalStall(USignal<uint8_t>*);
	void SetSignalIntr(USignal<uint8_t>*);
	
	//getters
	THFRiscV* GetCpu();
	UMemory* GetMem0();

	//getters for mems
	void SetMem0(UMemory*);
	
	USignal<uint32_t>* GetSignalHostTime();
	
	std::string ToString();
	std::string GetName();
};


#endif /* TROUTER_H */
