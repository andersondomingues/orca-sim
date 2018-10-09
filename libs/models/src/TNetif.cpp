/** 
 * This file is part of project URSA. More information on the project
 * can be found at URSA's repository at GitHub
 * 
 * http://https://github.com/andersondomingues/ursa
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

//std API
#include <iostream>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>

#include <TNetif.h>

TNetif::TNetif(std::string name) : TimedModel(name) {
      
    _comm_ack = nullptr;
    _comm_intr = nullptr;
    _comm_start = nullptr;
    _comm_status = nullptr;
	
	_mem1 = nullptr;
	_mem2 = nullptr;
    
    _ib = new UBuffer<FlitType>();
}

TNetif::~TNetif(){
    //nothing to do
}

void TNetif::Reset(){
    _recv_state = NetifSendState::WAIT;
    _send_state = NetifRecvState::WAIT;
    
    if(_comm_ack != nullptr) _comm_ack->Write(false);
    if(_comm_intr != nullptr) _comm_intr->Write(false);
    if(_comm_start != nullptr) _comm_start->Write(false);
    if(_comm_status != nullptr) _comm_status->Write(false);
    
    _ib->Reset();
}

void TNetif::SetOutputBuffer(UBuffer<FlitType>* ob){
	_ob = ob;
}

UBuffer<FlitType>* GetInputBuffer(){
	return _ib;
}


void SetMem1(UMemory* m){
	_mem1 = m;
}
void SetMem2(UMemory* m){
	_mem2 = m;
}


long long unsigned int TNetif::Run(){
    this->recvProcess();
    this->sendProcess();
    return 1; //takes only 1 cycle to change both states
}

void TNetif::recvProcess(){

	//recv state machine
	switch(_recv_state){
		
		//wait for data to arrive at input buffer. cannot
		//receive while interrupt is up
		case NetifRecvState::IDLE:
		
			//buffer has data and interruption flag is not set
			if(_ib->size() > 0 && _comm_intr == false){
				
				//removes length flit from buffer
				FlitType len_flit = _ib->top(); _ib->pop();
				
				//writes length flit to the memory
				_flits_to_recv = len_flit;
				_mem1->Write(0, &len_flit, 2);
				_next_addr = 2;
				
				//change state
				_recv_state = NetifRecvState::DATA_IN;
				
			}
			break;
						
		//copies words to memory (two-by-two, that is, 16-bits);
		case NetifRecvState::DATA_IN:
			
			if(_flits_to_recv > 0){
				
				//get next flit
				FlitType next = _ib->top(); _ib->pop();
				
				//writes to memory
				_mem->Write(_next_addr, (int8_t*)&next, 2);
				_next_addr += 2;
				_flits_to_recv--;
			}else{
				
				//no more flits to recv, change state
				_recv_state = NetifRecvState::INTR_CPU;
			}
			break;
		
		case NetifRecvState::INTR_CPU:
		
			//interrupts CPU
			_recv_intr->Write(1);
			_recv_state = NetifRecvState::WAIT;
			
		case NetifRecvState::WAIT:
			
			//wait until CPU finishes copying
			if(!_recv_intr->Read())
				_recv_state = NetifRecvState::IDLE;
				
		break;
	}
}

void TNetif::sendProcess(){	
	
	switch(_send_state){
		case NISendState::IDLE:
			if(_send_intr->Read() == 1){
				_send_state = NISendState::SETUP;
			}
			break;
		case NISendState::SETUP:
		
			//send header flit to router
			_ob->push(_ib->top());
			_ib->pop();
			
			//stores num of flits to send
			_words_to_send = _ib->top();
			_ob->push(_ib->top());
			_ib->pop();
			
			_send_state = NISendState::SEND;
			break;
		
		//burst send next flits
		case NISendState::SEND:
			if(_words_to_send == 0){
				_send_state = NISendState::DONE;
			}else{
				//send next
				_ob->push(_ib->top());
				_ib->pop();
			}
			break;
		case NISendState::DONE:
			_send_intr->Write(0); //down intr flag		
			_send_state = NISendState::IDLE;
			break;
	}
}
