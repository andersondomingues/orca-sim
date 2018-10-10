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
#include <queue>
#include <stdint.h>

//api includes
#include <UntimedModel.h>
#include <UComm.h>

template <typename T>
UComm<T>::UComm(std::string name, T default_value, uint32_t addr) : UntimedModel(name){
	_default = default_value;
	_addr = addr;
	this->Reset();
};

template <typename T>    
UComm<T>::~UComm(){
	//nothing to do
}

template <typename T>
T UComm<T>::Read(){
	return _val;
}
    
template <typename T>
void UComm<T>::Write(T val){
	_val = val;
}

template <typename T>
void UComm<T>::Reset(){
	_val = _default;
}

template <typename T>
uint32_t UComm<T>::GetAddr(){
	return _addr;
}