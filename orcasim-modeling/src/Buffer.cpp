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
#include "Buffer.hpp"

using orcasim::modeling::Buffer;

/**
 * @brief Instantiate a new buffer
 * @param name A unique name for the buffer (optional)
 */
template <typename T>
Buffer<T>::Buffer(std::string name, uint32_t capacity) : UntimedModel(name) {
    _name = name;
    _size = 0;
    _queue = new std::queue<T>();
    _capacity = capacity;
}

/**
 * @brief Dtor.
 */
template <typename T>
Buffer<T>::~Buffer() {
    // the underlying queue is the only object
    // with dynamic allocation
    delete(_queue);
}

/**
 * @brief Remove the element at the top of the buffer.
 */
template <typename T>
void Buffer<T>::pop() {
    // prevents underflow
    #ifdef BUFFER_UNDERFLOW_CHECKING
    if (_size == 0) {
        throw std::runtime_error(this->GetName() +
            ": unable to pop from an empty queue");
    }
    #endif

    _size--;
    _queue->pop();
}

/**
 * @brief Push an element to the bottom of the buffer.
 * @param e
 */
template <typename T>
void Buffer<T>::push(T e) {
    #ifdef BUFFER_OVERFLOW_CHECKING
    // prevents overflow
    if (_size == _capacity)
        throw std::runtime_error(this->GetName() +
            ": unable to push to a full queue.");
    #endif

    _size++;
    _queue->push(e);
}

/**
 * @brief Return the next element to be popped from the buffer.
 * @return the element
 */
template <typename T>
uint32_t Buffer<T>::capacity() {
    return _capacity;
}

/**
 * @brief Return the next element to be popped from the buffer.
 * @return the element
 */
template <typename T>
T Buffer<T>::top() {
    return _queue->front();
}

/**
 * @brief Return the number of elements store into underlying container
 * @return Number of elements
 */
template <typename T>
uint32_t Buffer<T>::size() {
    return _size;
}

/**
 * @brief Return the number of elements store into underlying container
 * @return Number of elements
 */
template <typename T>
uint32_t Buffer<T>::full() {
    return _size == _capacity;
}
