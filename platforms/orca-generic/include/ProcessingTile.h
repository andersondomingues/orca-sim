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
#ifndef __TProcessingTile_H
#define __TProcessingTile_H

//std API
#include <iostream>

//model API
#include <THellfireProcessor.h>
#include <TNetif.h>
#include <TRouter.h>
#include <UMemory.h>
#include <UComm.h>

//arch specifc
#include <Tile.h>

/* MEMORY LAYOUT
------------------- 0x40000000

       sram
      (~2MB)

------------------- 0x40400000
     empty space
------------------- 0x80000000
 COMM_NOC_ACK       0x80000001
 COMM_NOC_INTR      0x80000002
 COMM_NOC_START     0x80000003
      .....
------------------- 0x90000000
    mem1 (recv)
    (128 bytes)
------------------- 0x90000080
    mem2 (send)
    (128 bytes)
-------------------------------*/

//memory mapping
#define MEM0_SIZE 0x00400000 /* main memory */
#define MEM0_BASE 0x40000000

#define MEM1_SIZE 0x00000080 /* recv memory */
#define MEM1_BASE 0x90000000

#define MEM2_SIZE 0x00000080 /* send memory */
#define MEM2_BASE 0x90000080

#define COMM_NOC_ACK    0x80000001
#define COMM_NOC_INTR   0x80000002
#define COMM_NOC_START  0x80000003

/**
 * @class TProcessingTile
 * @author Anderson Domingues
 * @date 10/04/18
 * @file TProcessingTile.h
 * @brief This class models an entire processing element that contains
 * RAM memory (3x), DMA, NoC Router, HFRiscV core and an SPI interface. 
 */
class ProcessingTile : private Tile{

private:

	std::string _name;

	THellfireProcessor* _cpu; //hfrisv-core
	UMemory* _mem0; //main memory
	
public: 

	ProcessingTile(uint32_t x, uint32_t y);
	~ProcessingTile();
	
	//getters
	THellfireProcessor* GetCpu();
	UMemory* GetMem0();
	
	//getters for mems
	void SetMem0(UMemory*);
	
	std::string ToString();
};


#endif /* TROUTER_H */
