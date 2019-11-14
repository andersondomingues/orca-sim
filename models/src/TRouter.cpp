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
#include <sstream>
#include <iomanip>

#include <TRouter.h>


/**
 * @brief Get the name of the port of id equals to <port>
 * @param port the id of the port
 * @return and instance of std::string containing port's name */
std::string TRouter::GetPortName(int port){
	
	switch(port){
		case SOUTH: return "SOUTH";
		case NORTH: return "NORTH";
		case WEST:  return "WEST";
		case EAST:  return "EAST";
		default: return "LOCAL";
	}
	return "???";
}

/**
 * @brief Ctor.
 * @param name Name of the instance (proccess impl)
 * @param x_pos X coordinate (first part of router addr)
 * @param y_pos Y coordinate (second part of router addr) */
TRouter::TRouter(std::string name, uint32_t x_pos, uint32_t y_pos) : TimedModel(name){
   
    _x = x_pos;
    _y = y_pos;
		
	//for all ports, create a new input buffer; Note that data is bufferred by
	//input buffers, since output buffers come from somewhere else;
    for(int i = 0; i < 5; i++){
        std::string bname = GetName() +  ".IN-" + this->GetPortName(i);
        _ob[i] = nullptr;
        _ib[i] = new UBuffer<FlitType>(bname, BUFFER_CAPACITY);
    }
    
    this->Reset();
}

/**
 * @brief Free allocated memory if any
 */
TRouter::~TRouter(){
    
    #ifdef ROUTER_ENABLE_COUNTERS
    delete _counter_active;
    #endif
    
    for(int i = 0; i < 5; i++)
    	delete(_ib[i]);
}

void TRouter::Reset(){

    _round_robin = LOCAL; //starts checking on local port
    
    for(int i = 0; i < 5; i++){
    	_flits_to_send[i]  = 0;
    	_switch_control[i] = -1;
    }
	 
	 //TODO: reset buffers
}

uint32_t TRouter::GetRR(){
	return _round_robin;
}

/**
 * @brief Implementation of the Run method from
 * the Proccess abstract class.
 * @return The next time to schedule the event.*/
SimulationTime TRouter::Run(){
    
   	//std::cout << this->GetName() << std::endl;
    
	#ifdef ROUTER_ENABLE_COUNTERS
	bool is_active = false;
	#endif
    
	//CROSSBAR CONTROL: connect priority port to destination if it has any packet to 
	//send but is waiting for the destination to free
   	if(_ib[_round_robin]->size() > 0 && _switch_control[_round_robin] == -1){
    	
		//find the destination using the address in the first flit
		uint8_t target_port = this->GetRouteXY(_ib[_round_robin]->top()); 
		
		//printf("src: %d, trgt: %d\n", _round_robin, target_port);
		
		//check whether the destination port is bound to some other source port
		bool bound = false;
		
		for(int i = 0; i < 5; i++){
			if(_switch_control[i] == target_port){
				bound = true;
				break;
			}
		}

		//if the port is not bind, binds it to the source			
		if(!bound){
			_switch_control[_round_robin] = target_port; //set crossbar connection
			_flits_to_send[_round_robin] = -2; //-2 means "the size flit has not arrived yet"
		}
  	}
  	
    //drive flits into destination ports
	for(int i = 0; i < 5; i++){
	
    	//check whether the switch control is closed for some port
		if(_switch_control[i] != -1){
			
			//prevent routing to a non-existing router
			#ifdef ROUTER_PORT_CONNECTED_CHECKING
			if(_ob[_switch_control[i]] == nullptr){
				stringstream ss;
				ss << this->GetName() << ": unable to route to unknown port" << std::endl;
				std::runtime_error(ss.str());
			}
			
			if(_ib[i] == nullptr){
				stringstream ss;
				ss << this->GetName() << ": unable to route from unknown port" << std::endl;
				std::runtime_error(ss.str());
			}
			#endif
				
			//check whether the output is able to receive new flits. buffer must have some room
			if(!_ob[_switch_control[i]]->full() && _ib[i]->size() > 0){
			
				//if -2, we send the address flit
				if(_flits_to_send[i] == -2){

					_ob[_switch_control[i]]->push(_ib[i]->top()); //push one flit to destination port
					_flits_to_send[i] = -1;
					_ib[i]->pop(); //remove flit from source port
					
				//if -1, we set the size flit and send it 
				}else if(_flits_to_send[i] == -1){

					_flits_to_send[i] = _ib[i]->top();
					_ob[_switch_control[i]]->push(_ib[i]->top()); //push one flit to destination port
					_ib[i]->pop(); //remove flit from source port
				
				}else{

					_flits_to_send[i] -= 1;
					_ob[_switch_control[i]]->push(_ib[i]->top()); //push one flit to destination port
					_ib[i]->pop(); //remove flit from source port
				}

				#ifdef ROUTER_ENABLE_COUNTERS
				is_active = true;
				#endif

				//free port
				if(_flits_to_send[i] == 0) 
					_switch_control[i] = -1;
			
			}
		}
	}
	
	//round robin update
	_round_robin = (_round_robin + 1) % 5;
    	
	#ifdef ROUTER_ENABLE_COUNTERS
	if(is_active){
		_counter_active->Inc(1);
		is_active = false;
	}
	#endif
    	    
   return 1;
}

#ifdef ROUTER_ENABLE_COUNTERS
void TRouter::InitCounters(uint32_t couter_active_addr){
	_counter_active = new USignal<uint32_t>(couter_active_addr, GetName() + ".counters.active");
}

USignal<uint32_t>* TRouter::GetSignalCounterActive(){
	return this->_counter_active;
}

#endif

/**
 * @brief Calculate the port to route a given flit
 * @param flit to be routed
 * @return the port to where te packet must go*/
uint32_t TRouter::GetRouteXY(FlitType flit){
    
    FlitType tx = (flit & 0x00F0) >> 4;
    FlitType ty = flit & 0x000F;
    
    //printf("x:%d, y:%d, flit:0x%x\n", _x, _y, flit);
	
	//std::cout << this->GetName() << ":0x" << std::hex << flit << std::dec << ":(x, y) => (" << tx << "," << ty << ")" << std::endl;

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


std::string TRouter::ToString(){
	
	stringstream ss;
	ss << this->GetName() + ": ";
	
	for(int i = 0; i < 5; i++){
		if(_ob[i] != nullptr)
			ss << "{" << _ob[i]->GetName() << "} ";
	}
	
	return ss.str();
}