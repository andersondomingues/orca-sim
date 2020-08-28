/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software
 * http://https://github.com/andersondomingues/orca-mpsoc
 * http://https://github.com/andersondomingues/orca-tools
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
#ifndef ORCASIM_MODELING_INCLUDE_SIGNAL_HPP_
#define ORCASIM_MODELING_INCLUDE_SIGNAL_HPP_

// lib dependent includes
#include <stdint.h>
#include <iostream>
#include <vector>
#include <string>

// api includes
#include "UntimedModel.hpp"
#include "MemoryType.hpp"

namespace orcasim::modeling {

/**
 * The Signal class models a generic bus of width equals to the sizeof(T)
 */
template <typename T>
class Signal{
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
    Signal(T* t_ptr, uint32_t addr, std::string name);

     /**
     * @brief Constructor. Create new Signal using internal storage
     * @param name (optional) An arbitrary name for the instance.
     * @param addr (optinal) Address to which the bus is mapped in memory.
     */
    Signal(uint32_t addr, std::string name);

     /**
     * @brief Constructor. Create new Signal using internal storage
     * @param name (optional) An arbitrary name for the instance.
     */
    explicit Signal(std::string name);

    /**
     * @brief Destructor. 
     * @note DO NOT free _t_ptr by any means as this pointer must be 
     * freed by the class which allocated the space. 
     */
    ~Signal();

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
     * @param memptr External storage memory place
     * @param address Memory address if using memory maps
     * @param keep_val Set to true to keep current signal val
     */
    void MapTo(MemoryType* memptr, MemoryAddr address, bool keep_val = true);

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
     * @brief Set an address for this signal
     * */
    void SetAddress(MemoryAddr);

    /**
     * @brief Return the name of the Signal. 
     * @returns The name of the Signal or empty if no name has been set.
    */
    std::string GetName();
};

// Some of the most used instances. More can be added later.
// for larger data size, consider using a Memory instead.
template class Signal<bool>;  // wire
template class Signal<uint8_t>;   // mem word
template class Signal<uint16_t>;  // dmni/noc word
template class Signal<uint32_t>;  // proc word
template class Signal<float>;    // float cooprocessor output
template class Signal<uint64_t>;  // double word
template class Signal<int8_t>;   // mem word
template class Signal<int16_t>;  // dmni/noc word
template class Signal<int32_t>;  // proc word
template class Signal<int64_t>;  // double word

}  // namespace orcasim::modeling
#endif  // ORCASIM_MODELING_INCLUDE_SIGNAL_HPP_
