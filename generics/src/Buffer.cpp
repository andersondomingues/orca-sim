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
#include "Buffer.h"

template <typename T>
Buffer<T>::Buffer(std::string name){
    this->_name = name;
    this->_queue = new std::queue<T>();
}

template <typename T>
Buffer<T>::~Buffer(){
    //the underlying queue is the only object
    //with dynamic allocation
    delete(_queue);
}

template <typename T>
void Buffer<T>::pop(){
    _queue->pop();
}

template <typename T>
void Buffer<T>::push(T e){
    _queue->push(e);
}

template <typename T>
T Buffer<T>::top(){
    return _queue->front();
}

template <typename T>
uint32_t Buffer<T>::size(){
    return _queue->size();
}
