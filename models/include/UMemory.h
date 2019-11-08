/** 
 * This file is part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
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
#ifndef __GENERIC_RAM_H
#define __GENERIC_RAM_H

//simulation API
#include <UntimedModel.h>

//counter-specific definitions
#ifdef MEMORY_ENABLE_COUNTERS
#include <USignal.h>
#endif

#include <iostream>
#include <fstream>

#include <cstdlib>
#include <cstdint>

//Remeber: a memory is an arrays of int8_t, and
//addresses are 32-bit length
#define MemoryType int8_t
#define MemoryAddr uint32_t

// For performance reasons, a memory is implemented as   
// a vector of bytes (uint8_t). So, we opt for do not    
// encapsulate it inside a class. Thus, memory manipu-   
// lation is done through the static methods of the
// MemoryHelper class.
class UMemory: public UntimedModel
{

private:
   MemoryType* _mem;
   uint32_t _length;
	uint32_t _sram_base;

	#ifdef MEMORY_ENABLE_COUNTERS
	USignal<uint32_t>* _counter_nload;
	USignal<uint32_t>* _counter_nstore;
	#endif

public:

	#ifdef MEMORY_ENABLE_COUNTERS
	USignal<uint32_t>* GetSignalCounterLoad();
	USignal<uint32_t>* GetSignalCounterStore();
	void InitCounters(uint32_t store_counter_addr, uint32_t load_counter_addr);
	#endif

    /** Creates a new memory area.	
     * @param size: Total length of the memory are to be created.
     * @param wipe (optional): If <true> is passed, wipes the are after creating. */
	UMemory(std::string name, uint32_t size, uint32_t sram_base = 0, bool wipe = true, std::string binname = "");
	~UMemory();
    void Reset();  
    
    /** Writes data to a given memory location.
     * @param addr: Location to write to.
     * @param data: Pointer to the place that hold the data to be copied to the memory.
     * @param length: Length of the data to be copied into the memory location. */
    void Write(uint32_t addr, MemoryType* data, uint32_t length);

    /** Reads data from a given memory location. 
      * @param addr: Location to read from.
      * @param buffer: Buffer to where the read data will be copied.
      * @param length: Length of the data that will be copyied from memory to 
           the buffer. */
    void Read (uint32_t addr, MemoryType* buffer, uint32_t length);
		
    /** Loads the content of a given file to the memory. File contents are
           treated as binaray data.
      * @param filename: Name of the file to be read.
      * @param location: Location of the memory area in which the contents of 
           the loaded file will be put. */
    void LoadBin(std::string filename, uint32_t base, uint32_t size);

	/** Dump some memory as binary into a file
	  * @param filename: file to write to
	  * @param size: number of bytes to save
	  * @param addr: starting address (does not considers base) */
    void SaveBin(std::string filename, uint32_t base, uint32_t size);
    
    /** Write zeros to the whole memory area.
      * @param base: Starting address of the region to be wiped.
      * @param length: Number of memory positions to be overwritten. */
	void Wipe(uint32_t base, uint32_t length);
	
	/** Write zeroes to the whole memory area. This overload 
	  * wipes from _sram_base to _length. */
	void Wipe();
		
	/** Push the content of a memory are into sdtout.
	  * @param base: Initial address to start reading.
	  * @param size: Total length to be dumped. */
	void Dump(uint32_t base, uint32_t length);
	void Dump();
	
	/** Overrides operator [] for accessing internal mem data
	  * @param i: index of the internal array to be accessed
	  * @return the element stored in the given index */
	MemoryType &operator[](int i) {
	
		//TODO: validate behaviour
		return _mem[i];
	}
	    
	unsigned long long Run();    
	
	uint32_t GetBase(); //first address 
	uint32_t GetSize(); //number of addresses 
	
	//last address; NOTE THAT the last address is NOT (_base + _length), since the 
	//address zero still counts. Last address is (_base + _length - 1);
	uint32_t GetLastAddr(); //last address
	
	/**
	 * @brief Gets a pointer to given memory address
	 * @param addr Address to get the pointer from
	 * @return The pointer
	 */
	MemoryType* GetMap(uint32_t addr);
	
};

#endif