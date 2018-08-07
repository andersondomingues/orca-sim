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
#ifndef __BUFFER_H
#define __BUFFER_H

#include <iostream>
#include <queue>
#include <stdint.h>

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
class Buffer{

private:
    std::string _name;
    std::queue<T>* _queue;

public:
    /**
     * @brief Constructor. 
     * @param name (optional) An arbitrary name for the instance of Buffer.*/
    Buffer(std::string name = "");
    
    /**
     * @brief Destructor. Cleans dynamic allocated memory before disposing the object.*/
    ~Buffer();

    /**
     * @brief Peeks at the top of the buffer.
     * @return The object at the top of the buffer.*/
    T top();
    
    /**
     * @brief Removes the object at the front of the buffer. */
    void pop();
    
    /**
     * @brief Pushes an object to the back of the buffer */
    void push(T);
    
    /**
     * @brief Counts elements from the buffer.
     * @return The number of elements. */
    uint32_t size();
};


template class Buffer<uint8_t>;  //mem word
template class Buffer<uint16_t>; //dmni/noc word
template class Buffer<uint32_t>; //proc word


#endif
