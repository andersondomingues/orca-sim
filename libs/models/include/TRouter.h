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
#ifndef __TROUTER_H
#define __TROUTER_H

//std API
#include <iostream>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>

typedef uint16_t FlitType;

enum class RouterState{
    WORMHOLE, ROUNDROBIN
};

//routing ports
#define NORTH 0
#define WEST  1
#define SOUTH 2
#define EAST  3

#define LOCAL 4

class TRouter: public TimedModel{

private:
        uint32_t _round_robin;
        uint32_t _packets_to_send;
        uint32_t _target_port, _source_port;
        
        //address of the router
        uint32_t _x;
        uint32_t _y;
		
		//when this gla is enabled the router is 
		//routing the first flit af a packet. This is 
		//the only case in which the flit takes 4 cycles
		//to be sent.
		bool _is_first_flit;
        
        //state
        RouterState _state;
        
        //output buffers
        UBuffer<FlitType>* _ob[5];
        
        //input buffers
        UBuffer<FlitType>* _ib[5];        
public: 
        /**
         * @brief Get a pointer to one of the output buffers.
         * @param p The port to where the pointer will be pointing.
         * @return The pointer to the respective buffer. */
        UBuffer<FlitType>* GetOutputBuffer(uint32_t p);
        UBuffer<FlitType>* GetInputBuffer(uint32_t p);
        void SetInputBuffer(UBuffer<FlitType>* b, uint32_t port);
        
		/** Implementation of the Process' interface
		  * @return time taken for perming next cycle */
		unsigned long long Run();
		
        /** return this **/
		uint32_t GetRoute(uint32_t flit);
        
        /** Ctor. **/
		TRouter(string name, uint32_t x_pos, uint32_t y_pos);
	
		/** Dtor. **/
		~TRouter();
		
		void Reset();
};


#endif /* TROUTER_H */
