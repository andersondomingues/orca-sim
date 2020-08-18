/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software-tools
 * http://https://github.com/andersondomingues/orca-mpsoc
 *
 * Copyright (C) 2018-2020 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. 
******************************************************************************/
#ifndef __UMEMORY_H
#define __UMEMORY_H

//simulation API
#include <UntimedModel.h>

//std API
#include <iostream>
#include <fstream>

#include <cstdlib>
#include <cstdint>

//memory types
#include <MemoryType.h>

//counter-specific definitions
#ifdef MEMORY_ENABLE_COUNTERS
#include <USignal.h>
#endif

/**
 * @brief This class models a memory module. The module 
 * has no clock as it is an UntimedModel *
 */
class UMemory: public UntimedModel
{

private:	

	/**
	 * @brief The _mem attribute is an array of 
	 * MemoryType elements, where MemoryType is the 
	 * smalled memory unit that can be read or writen
	 * to the memory.
	 */
	MemoryType* _mem;

	/**
	 * @brief The _length attribute denotes the number 
	 * of cells of the memory module. The total number
	 * of memory space is given by sizeof(MemoryType) *
	 * _length (in bytes).
	 */
	MemoryAddr _length;

	/**
	 * @brief The _base attribute indicates the first 
	 * address of the memory module, an offset. Since
	 * this model can only address contiguous cells,
	 * one may use multiple memory models to emulate
	 * a non-contiguous space.
	 */
	MemoryAddr _base;

	#ifdef MEMORY_ENABLE_COUNTERS
	/**
	 * @brief The _counter_nload field stores the number 
	 * of load operations performed by the module. The 
	 * counter can be reset by writing zero to it.
	 */
	USignal<uint32_t>* _counter_nload;
	#endif

	#ifdef MEMORY_ENABLE_COUNTERS
	/**
	 * @brief The _counter_nstore field stores the number
	 * of store operations performed by the module. The
	 * counter can be reset by writing zero to it.
	 */
	USignal<uint32_t>* _counter_nstore;
	#endif

public:

	/**
	 * @brief Construct a new UMemory object.
	 * @param name A string name to identify the object. There is no 
	 * restriction on object names.
	 * @param size The number of cells to be emulated. The size of cell 
	 * is defined by the MemoryType type.
	 * @param base (optional) The base address for the emulated memory space. Default
	 * value is zero.
	 * @param wipe (optional) If set to true, erase memory cells after class instantia-
	 * tion, writing zero to each cell. Default behavior
	 * @param binname (optional)
	 */
	UMemory(std::string name, uint32_t size, uint32_t base = 0, bool wipe = false, std::string binname = "");

	/**
	 * @brief Destroy the UMemory object
	 */
	~UMemory();

	/**
	 * @brief Resets the memory module to its initial state. Since
	 * the model is stateless, this method has no effect on the data.
	 * To erase the memory, see <Wipe> method.
	 */
    void Reset() override;  
    
    /**
     * @brief Writes data to the memory. 
     * @param addr Address of the first cell to receive the data. If data
	 * is largen than a single cell, neighbor cells will be writen.
     * @param data A pointers of MemoryType that indicates the beggining of 
	 * data to be copied.
     * @param length Number of cells to write
     */
    void Write(uint32_t addr, MemoryType* data, uint32_t length);

    /** 
	 * @brief Reads data from a given memory location. 
     * @param addr: Location to read from.
     * @param buffer: Buffer to where the read data will be copied.
     * @param length: Length of the data that will be copyied from memory to 
     * the buffer. 
	 */
    void Read (uint32_t addr, MemoryType* buffer, uint32_t length);
		
    /** 
	 * @brief Loads the content of a given file to the memory. File contents are
     * treated as binaray data.
     * @param filename: Name of the file to be read.
     * @param location: Location of the memory area in which the contents of 
     * the loaded file will be put. 
	 */
    void LoadBin(std::string filename, MemoryAddr base, uint32_t size);

	/** 
	 * @brief Write the contents of memory cells into a binary file
	 * @param filename: file name, overwrites file is it exists already
	 * @param size: number of memory cells to save
	 * @param addr: address of the first cell to save
	 */
    void SaveBin(std::string filename, MemoryAddr base, uint32_t size);
    
    /**
     * @brief Write zeroes to the whole addressable memory space.
     */
	void Wipe();

	/**
	 * @brief Write zeroes to a region of the memory. 
	 * @param base The starting address, will be the first 
	 * address to be writen
	 * @param length Number of cells to write zeroes to. Cells 
	 * will follow the position of the base cell.
	 */
	void Wipe(MemoryAddr base, uint32_t length);
	
	/**
	 * @brief Display the contents of the memory on the output. Data
	 * will be displayed as hexadecimal values.
	 * @param base The first address to display
	 * @param length The number of cells to display
	 */
	void Dump(uint32_t base, uint32_t length);

	/**
	 * @brief Display the contents of the whole memory on the output.
	 */
	void Dump();
	
	/**
	 * @brief Shorthand access for the Read method using operator
	 * overloading on '[]' operator. On contrary of Read method, this
	 * operator is capable of returning only a single memory cell value.
	 * Please note that its counter for writing is not implemented.
	 * @param addr The address of the cell to read from
	 * @return MemoryType The value stored in the request cell 
	 */
	MemoryType &operator[](MemoryAddr addr) {
		return _mem[_base + addr];
	}
	    
	[[deprecated("UntimedModels should not have Run methods. This method will be removed soon.")]]
	unsigned long long Run();    
	
	/**
	 * @brief (getter) Gets the base address, which is the 
	 * first addressable memory cell in the module.
	 * @return MemoryAddr The address of the first memory cell.
	 */
	MemoryAddr GetBase();

	/**
	 * @brief (getter) Gets the size (alternatively, length) of the
	 * memory module, representing the number of addressable cells.
	 * @return MemoryAddr The number of addressable cells.
	 */
	MemoryAddr GetSize();

	/**
	 * @brief Get the address of the last addressable memory
	 * cell. The address is given by (base + length - 1).
	 * @return MemoryAddr The address of the last cell.
	 */
	MemoryAddr GetLastAddr();

	/**
	 * @brief Gets a pointer of MemoryType type that points to
	 * the cell in address <addr>. Although the pointer points to
	 * a single cell, neighbor cells can be accessed by incrementing
	 * on the pointer. Please note that this method bypassed the
	 * encapsuling methods Write and Read, and its use should be 
	 * avoided for data manipulation purpose.
	 * @param addr The addres to which the pointer must point to.
	 * @return MemoryType* A pointer to the given address.
	 */
	MemoryType* GetMap(MemoryAddr addr);

	#ifdef MEMORY_ENABLE_COUNTERS
	/**
	 * @brief (getter) Gets the signal object handling the 
	 * counter for number of performed load operations.
	 * @return USignal<uint32_t>* a pointer to the signal object.
	 */
	USignal<uint32_t>* GetSignalCounterLoad();
	#endif

	#ifdef MEMORY_ENABLE_COUNTERS
	/**
	 * @brief (getter) Gets the signal object handling the 
	 * counter for number of performed store operations.
	 * @return USignal<uint32_t>* a pointer to the signal object.
	 */
	USignal<uint32_t>* GetSignalCounterStore();
	#endif
};

#endif
