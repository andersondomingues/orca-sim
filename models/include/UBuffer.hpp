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
#ifndef MODELS_INCLUDE_UBUFFER_HPP_
#define MODELS_INCLUDE_UBUFFER_HPP_

#include <stdint.h>

// lib dependent includes
#include <iostream>
#include <queue>
#include <string>

// api includes
#include "UntimedModel.hpp"

#ifndef BUFFER_CAPACITY
#warning "BUFFER_CAPACITY not defined, defaulting to 16"
#define BUFFER_CAPACITY 16
#endif

/**
 * @class Buffer
 * @author Anderson Domingues
 * @date 07/31/18
 * @file Buffer.h
 * @brief Implement a generic buffer. This class implements the pattern "Container Adapter", 
 * and the underlying container is of std::queue type.
 * @param T The type of data being buffered.
 * the underlying type. */
template <typename T>
class UBuffer : public UntimedModel{
 private:
    std::string _name;
    std::queue<T>* _queue;
    uint32_t _size;
    uint32_t _capacity;

 public:
    /**
     * @brief Constructor. 
     * @param name (optional) An arbitrary name for the instance of Buffer.*/
    UBuffer(std::string name, uint32_t capacity);

    /**
     * @brief Destructor. Cleans dynamic allocated memory before disposing the object.*/
    ~UBuffer();

    /**
     * @brief Peeks at the top of the buffer.
     * @return The object at the top of the buffer.*/
    T top();

    /**
     * @brief Removes the object at the front of the buffer. */
    void pop();

    /**
     * @brief Returns TRUE when the buffer is full. */
    uint32_t full();

    /**
     * @brief Pushes an object to the back of the buffer */
    void push(T);

    /**
    * @brief Returns max size of the buffer */
    uint32_t capacity();

    /**
     * @brief Counts elements from the buffer.
     * @return The number of elements. */
    uint32_t size();
};

// Some of the most used instances. More can be added later.
template class UBuffer<uint8_t>;   // mem word
template class UBuffer<uint16_t>;  // dmni/noc word
template class UBuffer<uint32_t>;  // proc word

#endif  // MODELS_INCLUDE_UBUFFER_HPP_
