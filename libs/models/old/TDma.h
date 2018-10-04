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
#ifndef __TDMA_H
#define __TDMA_H

//std API
#include <iostream>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>

typedef uint16_t FlitType;

enum DmaState{ RR, NI_TO_MEM, MEM_TO_NI, ETH_TO_MEM, MEM_TO_ETH };


class TDma : public TimedModel{

private:
	UMemory* _mem;

public: 
	TDma();
	~TDma();
	
	ToMemory(uint8_t* addr, uint32_t size);
	FromMemory(uint8_t* )
        
};


#endif /* TDMA_H */
