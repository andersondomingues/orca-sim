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

//include peripheral addresses 
#include <MemoryMap.h>

#define MEM1_SIZE ORCA_MEMORY_SIZE_1 /* recv memory */
#define MEM1_BASE ORCA_MEMORY_BASE_1

#define MEM2_SIZE ORCA_MEMORY_SIZE_2 /* send memory */
#define MEM2_BASE ORCA_MEMORY_BASE_2

class Tile{

private:

	std::string _name;

	TRouter* _router; //hermes router
	USignal<uint32_t>* _signal_id; //	//self-id wire

public: 

	/*** ctor. & dtor. ***/
	Tile(uint32_t x, uint32_t y);
	~Tile();
	
	/*** getters ***/
	TRouter*   GetRouter();
	USignal<uint32_t>* GetSignalId();
	std::string GetName();
	
	/*** setters ***/
	void SetName(std::string);
};


#endif /* TILE_H */
