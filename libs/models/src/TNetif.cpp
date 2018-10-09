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
    
	
	_recv_state = NetifRecvState::READY;
	_flits_to_recv = 0;
	_next_recv_addr = 0;
	
    _send_state = NetifSendState::READY;
	_flits_to_send = 0;
    
    if(_comm_ack != nullptr) _comm_ack->Write(false);
    if(_comm_intr != nullptr) _comm_intr->Write(false);
    if(_comm_start != nullptr) _comm_start->Write(false);
    if(_comm_status != nullptr) _comm_status->Write(false);
    
    _ib->Reset();
}

void TNetif::SetOutputBuffer(UBuffer<FlitType>* ob){
	_ob = ob;
}

UBuffer<FlitType>* TNetif::GetInputBuffer(){
	return _ib;
}


//mems
void TNetif::SetMem1(UMemory* m){ _mem1 = m; }
void TNetif::SetMem2(UMemory* m){ _mem2 = m; }

//comms
void TNetif::SetCommAck(UComm<bool>* c){ _comm_ack = c; }
void TNetif::SetCommIntr(UComm<bool>* c){ _comm_intr = c; }
void TNetif::SetCommStart(UComm<bool>* c){ _comm_start = c; }
void TNetif::SetCommStatus(UComm<bool>* c){ _comm_status = c; }

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
		case NetifRecvState::READY:
		
			//buffer has data and interruption flag is not set
			if(_ib->size() > 0 && _comm_intr == false){
				
				//removes length flit from buffer
				FlitType len_flit = _ib->top(); _ib->pop();
				
				//writes length flit to the memory
				_flits_to_recv = len_flit;
				_mem1->Write(0, (int8_t*)&len_flit, 2);
				_next_recv_addr = 2;
				
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
				_mem1->Write(_next_recv_addr, (int8_t*)&next, 2);
				_next_recv_addr += 2;
				_flits_to_recv--;
			}else{
				
				//no more flits to recv, change state
				_recv_state = NetifRecvState::INTR_CPU;
			}
			break;
		
		case NetifRecvState::INTR_CPU:
		
			//interrupts CPU
			_comm_intr->Write(1);
			_recv_state = NetifRecvState::WAIT;
			
		case NetifRecvState::WAIT:
			
			//wait until CPU finishes copying. Then, disable both flags
			if(_comm_ack->Read()){
				_recv_state = NetifRecvState::READY;
				_comm_intr->Write(false);
				_comm_ack->Write(false);
			}
				
		break;
	}
}

void TNetif::sendProcess(){	
	
	switch(_send_state){
		
		case NetifSendState::READY:
			if(_comm_start->Read()){
				_send_state = NetifSendState::SETUP;
				_next_send_addr = 0;
			}
			break;
		case NetifSendState::SETUP:
		
			//send header flit to router
			FlitType header;
			_mem2->Read(0, (int8_t*)&header, 2);
			_ob->push(header);
			
			//get number of flits to send (and send it)
			_mem2->Read(2, (int8_t*)&header, 2);
			_ob->push(header);
			_flits_to_send = header;

			_send_state = NetifSendState::DATA_OUT;
			break;
		
		//burst send next flits
		case NetifSendState::DATA_OUT:
			
			if(_flits_to_send == 0){
				_send_state = NetifSendState::WAIT;
			}else{
				//send next
				FlitType data;
				_mem2->Read(_next_send_addr, (int8_t*)&data, 2);
				_ob->push(data);
				_next_send_addr += 2;
				_flits_to_send--;
			}
			break;
		case NetifSendState::WAIT:
			_comm_start->Write(0); //down start flag
			_send_state = NetifSendState::READY;
			break;
	}
}
