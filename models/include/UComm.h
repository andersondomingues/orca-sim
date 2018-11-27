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
#ifndef __UCOMM_H
#define __UCOMM_H

//lib dependent includes
#include <iostream>
#include <queue>
#include <stdint.h>

//api includes
#include <UntimedModel.h>

template <typename T>
class UComm : public UntimedModel{

private:
    std::string _name;
    T _default;
    T _val;
	uint32_t _addr;

public:
    /**
     * @brief Constructor. 
     * @param name (optional) An arbitrary name for the instance of Buffer.*/
    UComm(std::string name, T default_value, uint32_t addr = 0x0);
    
    /**
     * @brief Destructor. Cleans dynamic allocated memory before disposing the object.*/
    ~UComm();

    /**
     * @brief Peeks at the top of the buffer.
     * @return The object at the top of the buffer.*/
    T Read();
    
    /**
     * @brief Removes the object at the front of the buffer. */
    void Write(T val);
    
	
	/**
	 * @brief Empties the queue */
	void Reset();
	uint32_t GetAddr();
};

//Some of the most used instances. More can be added later.
template class UComm<bool>;  //wire
template class UComm<uint8_t>;  //mem word
template class UComm<uint16_t>; //dmni/noc word
template class UComm<uint32_t>; //proc word
template class UComm<uint64_t>; //double word
template class UComm<int8_t>;  //mem word
template class UComm<int16_t>; //dmni/noc word
template class UComm<int32_t>; //proc word
template class UComm<int64_t>; //double word

//for larger data size, consider using a UMemory instead.

#endif /* UComm_H */