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
#include <stdint.h>

#include <iostream>
#include <vector>
#include <algorithm>

// api includes
#include "UntimedModel.hpp"
#include "USignal.hpp"

/**
 * @brief Instiate a new bus with external storage (can be changed later via 
 * "MapTo(&)")
 * @param name A unique name to the bus (optional)
 * @param default_value A value to be read in case none has been set yet
 * @param addr A memory base to be used within memory mapping
 */
template <typename T>
USignal<T>::USignal(std::string name) {
    _t_ptr  = &_t_storage;
    _t_name = name;
}

// @TODO(ad): rework class to inherit from UntimedModel
// @TODO(ad): turn multiple construction into a single on using optional params

/**
 * @brief Instiate a new bus with external storage (can be changed later via "MapTo()")
 * @param t_ptr
 * @param addr
 * @param name
 */
template <typename T>
USignal<T>::USignal(T* t_ptr, uint32_t addr, std::string name) {
    _t_ptr  = t_ptr;
    _t_addr = addr;
    _t_name = name;
}

template <typename T>
USignal<T>::USignal(uint32_t addr, std::string name) {
    _t_ptr = &_t_storage;
    _t_addr = addr;
    _t_name = name;
}

/**
* @brief Maps current Signal to the internal storage
* @param keep_val Copies current value to internal storage
*/
template <typename T>
void USignal<T>::MapTo(bool keep_val) {
    // copies current value before changing pointers
    if (keep_val) {
        T curr_val;
        curr_val = this->Read();
        _t_ptr = &_t_storage;
        this->Write(curr_val);
    } else {
        _t_ptr = &_t_storage;
    }
}

/**
* @brief Maps current Signal to an external storage and updates internal reference.
* @param memptr Location to shadown the access in
* @param address to be used if mapping to memory
* @param keep_val set to true to keep current signal value
*/
template <typename T>
void USignal<T>::MapTo(MemoryType* memptr, MemoryAddr address, bool keep_val) {
    // copies current value before changing pointers
    if (keep_val)
        *(reinterpret_cast<MemoryType*>(memptr)) = this->Read();

    _t_ptr = reinterpret_cast<T*>(memptr);
    _t_addr = address;
}

/**
 * @brief Dtor.
 */
template <typename T>
USignal<T>::~USignal() { }

/**
 * @brief Read the value stored into the bus
 * @return the value
 */
template <typename T>
T USignal<T>::Read() {
    return *_t_ptr;
}

/**
 * @brief Set the value of the bus
 * @param val the value
 */
template <typename T>
void USignal<T>::Write(T val) {
    *_t_ptr = val;
}

/**
 * @brief Set the value of the bus
 * @param val the value
 */
template <typename T>
void USignal<T>::Inc(T val) {
    *_t_ptr = (*_t_ptr) + val;
}

/**
 * @brief Set the value of the bus
 * @param val the value
 */
template <typename T>
void USignal<T>::Dec(T val) {
    *_t_ptr = (*_t_ptr) - val;
}

/**
 * @brief Get the memory mapping address
 * @return the address
 */
template <typename T>
uint32_t USignal<T>::GetAddress() {
    return _t_addr;
}

/**
 * @brief Get the name of the Signal
 * @return the name (empty string if empty)
 */
template <typename T>
std::string USignal<T>::GetName() {
    return _t_name;
}

/**
 * @brief Set an address for the signal
 */
template <typename T>
void USignal<T>::SetAddress(MemoryAddr addr) {
    _t_addr = addr;
}
