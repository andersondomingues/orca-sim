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
#include <cstdlib>
#include <TRouter.h>

/**
 * @brief Ctor.
 * @param name Name of the instance (proccess impl)
 * @param x_pos X coordinate (first part of router addr)
 * @param y_pos Y coordinate (second part of router addr) */
TRouter::TRouter(std::string name, uint32_t x_pos, uint32_t y_pos) : TimedModel(name){
   
    _x = x_pos;
    _y = y_pos;
	_is_first_flit = false; //starts in roundrobin mode, not flit to be routed
    
    for(int i = 0; i < 5; i++){
        std::string bname = "(" + std::to_string(_x) + "," + std::to_string(_y) + ").IN" + std::to_string(i);
        _ob[i] = nullptr;
        _ib[i] = new UBuffer<FlitType>(bname);
    }
    
    this->Reset();
}

void TRouter::Reset(){
    _round_robin = LOCAL; //starts checking on local port
    _state = RouterState::ROUNDROBIN;
    _packets_to_send = 0;
}

/**
 * @brief Implementation of the Run method from
 * the Proccess abstract class.
 * @return The next time to schedule the event.*/
unsigned long long TRouter::Run(){
    
    switch(_state){
		
		//In ROUNDROBIN state, the router wait for some of the 
		//input ports to have packages to be sent. The order is, 
		//of course, defined by the RR algorithm. When some of the 
		//ports have packages to be sent, the router changes to 
		//WHORMHOLE state.
		case RouterState::ROUNDROBIN:
        {
            UBuffer<FlitType>* tb = _ib[_round_robin];

            //prevent from serving unconnected ports (e.g. border)
            if(tb != nullptr){
				
				//if has packet to send
                if(tb->size() > 0){  
					
                    _state = RouterState::WORMHOLE;
                    _source_port = _round_robin;
                    _target_port = this->GetRoute(tb->top()); 
                    
                    std::cout << "[" << _target_port << "]" << endl;
                    
                    _is_first_flit = true;
					
                    //alternativelly:
					//_packets_to_send = tb->size(); 
                    _packets_to_send = tb->top() & 0x0000FFFF;
                }
            }
			
			//get next port
            _round_robin++;
            _round_robin = _round_robin % 5; 
            
            break;
        } 
  
		//in WORMHOLE state, the router keeps sending flits until there
		//is no more flits to be sent. When the last flit is sent, the 
		//router returns to ROUNDROBIN state.
        case RouterState::WORMHOLE:
        {
			_is_first_flit = false;
			
            //if packets to be sent, 
            UBuffer<FlitType>* ob = _ob[_target_port];
            UBuffer<FlitType>* ib = _ib[_source_port];
            
            ob->push(ib->top());
            ib->pop();
            
            _packets_to_send--;
            
            if(_packets_to_send == 0)
                _state = RouterState::ROUNDROBIN;

            break;
        }
    }
	
	//First flit takes 4 cycles to be sent due the time consumed 
	//by the routing algorithm. When the rr finds no canditate to
	//send flits or the flit is other than the first, it takes only
	//one cycle to happen.
	return (_is_first_flit) ? 4 : 1;
}

/**
 * @brief Calculate the port to route a given flit
 * @param flit to be routed
 * @return the port to where te packet must go*/
uint32_t TRouter::GetRoute(FlitType flit){
    
    FlitType tx = (flit & 0xFF00) >> 8;
    FlitType ty = (flit & 0x00FF);

    //if X=0, then route "vertically" (Y)
    if(_x == tx){
    
        return (_y == ty)
            ? LOCAL
            : (_y > ty)
                ? SOUTH
                : NORTH;      
    //else route X
    }else{

        return (_x > tx)
            ? WEST
            : EAST;
    }
}

/**
 * @brief Free allocated memory if any
 */
TRouter::~TRouter(){
    //nothing to do here
}


/**
 * @brief Get a pointer to one of the output buffers.
 * @param r The port from which get the pointer.
 * @return A pointer to the buffer.*/
UBuffer<FlitType>* TRouter::GetOutputBuffer(uint32_t r){
    return _ob[r];
}

UBuffer<FlitType>* TRouter::GetInputBuffer(uint32_t r){
    return _ib[r];
}

void TRouter::SetOutputBuffer(UBuffer<FlitType>* b, uint32_t port){
    _ob[port] = b;
}
