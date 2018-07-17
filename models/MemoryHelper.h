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

#include <iostream>
#include <fstream>

#include <cstdlib>
#include <cstdint>

// For performance reasons, a memory is implemented as   
// a vector of bytes (uint8_t). So, we opt for do not    
// encapsulate it inside a class. Thus, memory manipu-   
// lation is done through the static methods of the
// MemoryHelper class.
#define MemoryType int8_t *


class MemoryHelper{

  public: //<<-------------- all methods here are public
		
	/** Creates a new memory area.	
     * @param size: Total length of the memory are to be created.
     * @param wipe (optional): If <true> is passed, wipes the are after creating. */
    static MemoryType Create(uint32_t size, bool wipe = false);

    /** Writes data to a given memory location.
     * @param mem: Pointer to the memory area to be written.
     * @param addr: Location to write to.
     * @param data: Pointer to the place that hold the data to be copied to the memory.
     * @param length: Length of the data to be copied into the memory location. */
    static void Write(MemoryType mem, uint32_t addr, uint8_t* data, uint32_t length);

    /** Reads data from a given memory location. 
      * @param mem: Pointer to the memory area to be read.
      * @param addr: Location to read from.
      * @param buffer: Buffer to where the read data will be copied.
      * @param length: Length of the data that will be copyied from memory to 
           the buffer. */
    static void Read (MemoryType mem, uint32_t addr, uint8_t* buffer, uint32_t length);
		
    /** Loads the content of a given file to the memory. File contents are
           treated as binaray data.
      * @param mem: Pointer to the memory are to be written.
      * @param filename: Name of the file to be read.
      * @param location: Location of the memory area in which the contents of 
           the loaded file will be put. */
    static void LoadBin(MemoryType mem, std::string filename, uint32_t size, uint32_t addr = 0);
		
    /** Write zeros to the whole memory area.
      * @param mem: Pointer to the memory area to be wiped.
      * @param base: Starting address of the region to be wiped.
      * @param length: Number of memory positions to be overwritten. */
	static void Wipe(MemoryType mem, uint32_t base, uint32_t length);
};

#endif