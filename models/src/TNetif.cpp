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
#include <sstream>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>

#include <TNetif.h>

//int xyz = 0;

TNetif::TNetif(std::string name) : TimedModel(name) {
      
    _signal_ack = nullptr;
    _signal_intr = nullptr;
    _signal_start = nullptr;
	
	_mem1 = nullptr;
	_mem2 = nullptr;
    
    _ib = new UBuffer<FlitType>(name + ".IN", NI_BUFFER_LEN);
	
	this->Reset();
}

TNetif::~TNetif(){
    delete _ib;
}

void TNetif::Reset(){
    	
	_recv_state = NetifRecvState::READY;
	_flits_to_recv = 0;
	
    _send_state = NetifSendState::READY;
	_flits_to_send = 0;
}

void TNetif::SetOutputBuffer(UBuffer<FlitType>* ob){
	_ob = ob;
}

UBuffer<FlitType>* TNetif::GetInputBuffer(){
	return _ib;
}

//state getters
NetifRecvState TNetif::GetRecvState(){
	return _recv_state;
}

NetifSendState TNetif::GetSendState(){
	return _send_state;
}


//recv mem
void TNetif::SetMem1(UMemory* m1){ 
	_mem1 = m1; 
	_next_recv_addr = _mem1->GetBase();
}

//send mem
void TNetif::SetMem2(UMemory* m2){
	_mem2 = m2; 
	_next_send_addr = _mem2->GetBase();
}

//signals
void TNetif::SetSignalAck(USignal<int8_t>* c){ _signal_ack = c; }
void TNetif::SetSignalIntr(USignal<int8_t>* c){ _signal_intr = c; }
void TNetif::SetSignalStart(USignal<int8_t>* c){ _signal_start = c; }
void TNetif::SetSignalStatus(USignal<int8_t>* c){ _signal_status = c; }

USignal<int8_t>* TNetif::GetSignalAck(){ return _signal_ack; }
USignal<int8_t>* TNetif::GetSignalIntr(){ return _signal_intr; }
USignal<int8_t>* TNetif::GetSignalStart(){ return _signal_start; }
USignal<int8_t>* TNetif::GetSignalStatus(){ return _signal_status; }

SimulationTime TNetif::Run(){
    this->recvProcess();
    this->sendProcess();   
    
    return 1; //takes only 1 cycle to change both states
}

void TNetif::recvProcess(){

	//recv state machine
	switch(_recv_state){
		
		//STATE 1: wait for data to arrive at the buffer. 
		case NetifRecvState::READY:{
		
			//buffer has data and interruption flag is not set
			//ack must be down as well
			if(_ib->size() > 0){
				
				//writes addr header to mem
				_next_recv_addr = _mem1->GetBase();
				
				FlitType flit = _ib->top(); 
				_ib->pop();
				_mem1->Write(_next_recv_addr, (int8_t*)&flit, 2);
				
				//std::cout << std::hex << flit << std::endl;
				
				_next_recv_addr += sizeof(FlitType);
				_recv_state = NetifRecvState::LENGTH;	
			}
			
		}break;		
		
		//check for the length of arrived data. Cannot greater than buffer size
		case NetifRecvState::LENGTH:{
			
			if(_ib->size() > 0){
				
				FlitType len_flit = _ib->top(); 
				_ib->pop();
				
				_mem1->Write(_next_recv_addr, (int8_t*)&len_flit, 2);
				_next_recv_addr += sizeof(FlitType);
				
				_flits_to_recv = len_flit;
				_recv_state = NetifRecvState::DATA_IN;
			}
			
		}break;

		//copies words to memory (two-by-two, that is, 16-bits);
		case NetifRecvState::DATA_IN:{
			
			//no more flits to recv, change state
			if(_flits_to_recv == 0){

				_signal_intr->Write(0x1); //interrupts CPU
				_recv_state = NetifRecvState::INTR_AND_WAIT;
			
			}else if(_ib->size() > 0){
				
				//get next flit
				FlitType next = _ib->top(); 
				_ib->pop();
				
				//writes to memory
				_mem1->Write(_next_recv_addr, (int8_t*)&next, 2);
				
				_next_recv_addr += sizeof(FlitType);
				_flits_to_recv--;
			}
			
		} break;
				
		//wait until CPU finishes copying. then, disable interruption
		case NetifRecvState::INTR_AND_WAIT:{
			
			if(_signal_ack->Read() == 0x1){
				_signal_intr->Write(0x0);//lowers interruption
				_recv_state = NetifRecvState::FLUSH;
			}
			
		} break;
		
		//wait for cpu to acknowdledge the operation (by lowering the acknowledge)
		case NetifRecvState::FLUSH:{
		
			if(_signal_ack->Read() == 0x0)
				_recv_state = NetifRecvState::READY;
				
		} break;
	}
}

void TNetif::sendProcess(){	

	switch(_send_state){
			
		case NetifSendState::READY:{
			
			//cpu asked to start AND router has enough room to receive flits
			if(_signal_start->Read() == 0x1 && _ob->size() < _ob->capacity()){
				
				_next_send_addr = _mem2->GetBase();
			
				//push first packet to router's input buffer
				FlitType header;
				_mem2->Read(_next_send_addr, (int8_t*)&header, sizeof(FlitType));

				_ob->push(header);
				_next_send_addr += sizeof(FlitType);
			
				//change state
				_send_state = NetifSendState::LENGTH;
			
				//std::cout << GetName() << ": READY OK" << std::endl;
			
			}
			
		} break;

		case NetifSendState::LENGTH:{

			//check whether the router can receive another flit			
			if(_ob->size() < _ob->capacity()){
			
				FlitType flit;

				//push length flit into router's buffer
				_mem2->Read(_next_send_addr, (int8_t*)&flit, sizeof(FlitType));
				_ob->push(flit);
				_next_send_addr += sizeof(FlitType);
			
				_flits_to_send = flit;
				_send_state = NetifSendState::DATA_OUT;
			}
			
		}break;
		
		//burst send next flits
		case NetifSendState::DATA_OUT:{

			//no more flits to send
			if(_flits_to_send == 0) {

				_send_state = NetifSendState::READY;
				_signal_start->Write(0); //start = 0 means "ready to send another packet"
				
				//std::cout << GetName() << ": DATA_OUT OK" << std::endl;
				
			//in case not all flits where sent, keep sending
			}else if(_ob->size() < _ob->capacity()){

				FlitType flit;
				_mem2->Read(_next_send_addr, (int8_t*)&flit, 2);
				_ob->push(flit);
				_next_send_addr += sizeof(FlitType);
				
				_flits_to_send--;
			}

		} break;

	}
}
