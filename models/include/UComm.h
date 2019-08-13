/** 
 * This part of project URSA. More information on the project
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
#ifndef __UCOMM_H
#define __UCOMM_H

//lib dependent includes
#include <iostream>
#include <queue>
#include <stdint.h>

//api includes
#include <UntimedModel.h>

/**
 * The UComm class models a generic bus of width equals to the sizeof(T)
 */
template <typename T>
class UComm{

private:
	/** pointer to the place where the bus data will be stored */
   T* _t_ptr;
	 
	/** internal storage (necessary when mmio is not used) */
	T _t_storage;
    
   /** an optional name to identify this bus during runtime */
	std::string _t_name;
    
	/** a memory address in case this is mapped using mmio */	
	uint32_t _t_addr;

public:
    /**
     * @brief Constructor. Creates a new comm using external storage
     * @param t_ptr pointer to the location of the value of the bus.
     * @param name (optional) An arbitrary name for the instance.
     * @param addre (optinal) Address to which the bus is mapped in memory.
     */
	 UComm(T* t_ptr, uint32_t addr, std::string name);
	
	 /**
     * @brief Constructor. Create new comm using internal storage
     * @param name (optional) An arbitrary name for the instance.
     * @param addre (optinal) Address to which the bus is mapped in memory.
     */
	 UComm(uint32_t addr, std::string name);
    
    /**
     * @brief Destructor. 
     * @note DO NOT free _t_ptr by any means as this pointer must be 
     * freed by the class which allocated the space. 
     */
    ~UComm();

    /**
     * @brief Get the last value writen to the bus.
     * @return A value of type T.
     */
    T Read();
	 
	 /**
	  * @brief Maps current comm to the internal storage
	  */
	 void MapTo(bool keep_val = true);
	 
	 /**
	  * @brief Maps current comm to an external storage, updates internal reference 
	  * and sets a new address.
	  * @param m External storage address
	  * @param p Reference address
	  */
	 void MapTo(T* m, uint32_t p, bool keep_val = true);
	 
    /**
     * @brief Writes some value to the bus
     * @param val Value to be writen to the bus
     */
    void Write(T val);
    
	/**
	 * @brief Increments the value of the bus by the given value.
	 * @param val Value to be added to the current value of the bus 
	 */
	void Inc(T val);

	/**
	 * @brief Descrements the value of the bus by the given value.
	 * @param val Value to besubtracted from the current value of the bus 
	 */
	void Dec(T val);
	
	/**
	 * @brief Return the addres to which this comm is mapped. 
	 * @returns The address of mmio (zero if not mapped)
	*/
	uint32_t GetAddress();
	
	/**
	 * @brief Return the name of the comm. 
	 * @returns The name of the comm or empty if no name has been set.
	*/
	std::string GetName();
};

//Some of the most used instances. More can be added later.
//for larger data size, consider using a UMemory instead.
template class UComm<bool>;  //wire
template class UComm<uint8_t>;  //mem word
template class UComm<uint16_t>; //dmni/noc word
template class UComm<uint32_t>; //proc word
template class UComm<uint64_t>; //double word
template class UComm<int8_t>;  //mem word
template class UComm<int16_t>; //dmni/noc word
template class UComm<int32_t>; //proc word
template class UComm<int64_t>; //double word



#endif /* UComm_H */
