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
#ifndef __USIGNAL_H
#define __USIGNAL_H

//lib dependent includes
#include <iostream>
//#include <queue>
#include <vector>
#include <stdint.h>

//api includes
#include <UntimedModel.h>

/**
 * This interface permit all Signals to share a same base type, thus being
 * able to be stored in a single container without forcing dynamic inheritance.
 */
class ISignal{ /** dummy **/ 

public:
	static std::vector<ISignal*> signals;

};

/**
 * The USignal class models a generic bus of width equals to the sizeof(T)
 */
template <typename T>
class USignal : public ISignal {

private:
	/** pointer to the place where the bus data will be stored */
    volatile T* _t_ptr;
	 
	/** internal storage (necessary when mmio is not used) */
	volatile T _t_storage;
    
    /** an optional name to identify this bus during runtime */
	std::string _t_name;
    
	/** a memory address in case this is mapped using mmio */	
	volatile uint32_t _t_addr;
	
public:

    /**
     * @brief Constructor. Creates a new Signal using external storage
     * @param t_ptr pointer to the location of the value of the bus.
     * @param name (optional) An arbitrary name for the instance.
     * @param addre (optinal) Address to which the bus is mapped in memory.
     */
	 USignal(T* t_ptr, uint32_t addr, std::string name);
	
	 /**
     * @brief Constructor. Create new Signal using internal storage
     * @param name (optional) An arbitrary name for the instance.
     * @param addre (optinal) Address to which the bus is mapped in memory.
     */
	 USignal(uint32_t addr, std::string name);
    
    /**
     * @brief Destructor. 
     * @note DO NOT free _t_ptr by any means as this pointer must be 
     * freed by the class which allocated the space. 
     */
    ~USignal();

    /**
     * @brief Get the last value writen to the bus.
     * @return A value of type T.
     */
    T Read();
	 
	 /**
	  * @brief Maps current Signal to the internal storage
	  */
	 void MapTo(bool keep_val = true);
	 
	 /**
	  * @brief Maps current Signal to an external storage, updates internal reference 
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
	 * @brief Return the addres to which this Signal is mapped. 
	 * @returns The address of mmio (zero if not mapped)
	*/
	uint32_t GetAddress();
	
	/**
	 * @brief Return the name of the Signal. 
	 * @returns The name of the Signal or empty if no name has been set.
	*/
	std::string GetName();
};

//Some of the most used instances. More can be added later.
//for larger data size, consider using a UMemory instead.
template class USignal<bool>;  //wire
template class USignal<uint8_t>;  //mem word
template class USignal<uint16_t>; //dmni/noc word
template class USignal<uint32_t>; //proc word
template class USignal<uint64_t>; //double word
template class USignal<int8_t>;  //mem word
template class USignal<int16_t>; //dmni/noc word
template class USignal<int32_t>; //proc word
template class USignal<int64_t>; //double word


#endif /* USIGNAL_H */
