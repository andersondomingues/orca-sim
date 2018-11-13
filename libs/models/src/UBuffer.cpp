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
#include <UBuffer.h>

/**
 * @brief Instantiate a new buffer
 * @param name A unique name for the buffer (optional)
 */
template <typename T>
UBuffer<T>::UBuffer(std::string name) : UntimedModel(name){
    _name = name;
    _queue = new std::queue<T>();
}

/**
 * @brief Dtor.
 */
template <typename T>
UBuffer<T>::~UBuffer(){
    //the underlying queue is the only object
    //with dynamic allocation
    delete(_queue);
}

/**
 * @brief Remove the element at the top of the buffer.
 */
template <typename T>
void UBuffer<T>::pop(){
    _queue->pop();
}

/**
 * @brief Push an element to the bottom of the buffer.
 * @param e
 */
template <typename T>
void UBuffer<T>::push(T e){
    _queue->push(e);
}

/**
 * @brief Return the next element to be popped from the buffer.
 * @return the element
 */
template <typename T>
T UBuffer<T>::top(){
    return _queue->front();
}

/**
 * @brief Return the number of elements residing into the buffer
 * @return the number of elements
 */
template <typename T>
uint32_t UBuffer<T>::size(){
    return _queue->size();
}

/**
 * @brief Delete the internal buffer and instantiate a new one with zero
 * elements
 */
template <typename T>
void UBuffer<T>::Reset(){
	delete(_queue);
	_queue = new std::queue<T>();
}