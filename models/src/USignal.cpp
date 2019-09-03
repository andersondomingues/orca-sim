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
//lib dependent includes
#include <iostream>
//#include <queue>
#include <vector>
#include <algorithm>
#include <stdint.h>

//api includes
#include <UntimedModel.h>
#include <USignal.h>


std::vector<ISignal*> ISignal::signals = std::vector<ISignal*>();

/**
 * @brief Instiate a new bus with external storage (can be changed later via "MapTo(&)")
 * @param name A unique name to the bus (optional)
 * @param default_value A value to be read in case none has been set yet
 * @param addr A memory base to be used within memory mapping
 */
template <typename T>
USignal<T>::USignal(uint32_t addr, std::string name){
	_t_ptr  = &_t_storage;
	_t_addr = addr;
	_t_name = name;
	
	//register this instance with the global observer
	ISignal::signals.push_back(this);
};

/**
 * @brief Instiate a new bus with external storage (can be changed later via "MapTo()")
 * @param t_ptr
 * @param addr
 * @param name
 */
template <typename T>
USignal<T>::USignal(T* t_ptr, uint32_t addr, std::string name){
	_t_ptr  = t_ptr;
	_t_addr = addr;
	_t_name = name;
};

/**
* @brief Maps current Signal to the internal storage
* @param keep_val Copies current value to internal storage
*/
template <typename T>
void USignal<T>::MapTo(bool keep_val){

	//copies current value before changing pointers
	if(keep_val){
	
		T curr_val;	
		curr_val = this->Read();
		_t_ptr = &_t_storage;
		this->Write(curr_val);

	}else{
		_t_ptr = &_t_storage;
	}
}

/**
* @brief Maps current Signal to an external storage and updates internal reference.
* @param addr
* @param m
*/
template <typename T>
void USignal<T>::MapTo(T* addr, uint32_t p, bool keep_val){

	//copies current value before changing pointers
	if(keep_val) 
		*addr = this->Read();

	_t_ptr = addr;
	_t_addr = p;
}

/**
 * @brief Dtor.
 */
template <typename T>    
USignal<T>::~USignal(){
	
	//unregister this instance with the global observer
	std::vector<ISignal*>::iterator it;
	
	it = std::find(
		ISignal::signals.begin(),
		ISignal::signals.end(),
		this);
		
	if(it != ISignal::signals.end()){
		ISignal::signals.erase(it);	
	}else{ 
		std::cout << "could not find signal in global observer" << std::endl;
	}
}

/**
 * @brief Read the value stored into the bus
 * @return the value
 */
template <typename T>
T USignal<T>::Read(){
	return *_t_ptr;
}
    
/**
 * @brief Set the value of the bus
 * @param val the value
 */
template <typename T>
void USignal<T>::Write(T val){
	*_t_ptr = val;
}

/**
 * @brief Set the value of the bus
 * @param val the value
 */
template <typename T>
void USignal<T>::Inc(T val){
	*_t_ptr = (*_t_ptr) + val;
}

/**
 * @brief Set the value of the bus
 * @param val the value
 */
template <typename T>
void USignal<T>::Dec(T val){
	*_t_ptr = (*_t_ptr) + val;
}

/**
 * @brief Get the memory mapping address
 * @return the address
 */
template <typename T>
uint32_t USignal<T>::GetAddress(){
	return _t_addr;
}

/**
 * @brief Get the name of the Signal
 * @return the name (empty string if empty)
 */
template <typename T>
std::string USignal<T>::GetName(){
	return _t_name;
}