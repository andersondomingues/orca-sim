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

TNetif::TNetif(std::string name, uint32_t addr) : TimedModel(name) {
	_ob = nullptr;
	_ib = new UBuffer<FlitType>();
	_intr = false;
	_base_addr = addr;
}

UBuffer<FlitType>* TNetif::GetOutputBuffer(){
	return _ob;
}

UBuffer<FlitType>* TNetif::GetInputBuffer(){
	return _ib;
}
        
void TNetif::SetOutputBuffer(UBuffer<FlitType>* b){
	_ob = b;
}


TNetif::~TNetif(){
	delete(_ib);
}

void TNetif::Reset(){
	//_ib->clear();	
}

long long unsigned int TNetif::Run(){

	this->recvProcess();
	this->sendProcess();
	return 1;
}

void TNetif::SetBaseAddr(uint32_t addr){
	_base_addr = addr;	
}

void TNetif::recvProcess(){

	//recv state machine
	switch(_recv_state){
		
		//wait for data to arrive at input buffer. cannot
		//receive while interrupt is up
		case NIRecvState::IDLE:
			if(_ib->size() >= 2 && _intr == 0){
				_recv_state = NIRecvState::WRITE_DATA;
				_ib->pop(); //remove destination flit
			}
			break;
				
		//calculate number of words to copy
		case NIRecvState::WRITE_DATA:
		
			_words_to_copy = _ib->top();
			_next_addr = _base_addr;
			_ib->pop(); //remove size flit from buffer			
			_recv_state = NIRecvState::RECV_TO_MEM;
			break;
		
		//copies words to memory (two-by-two, that is, 16-bits);
		case NIRecvState::RECV_TO_MEM:
			
			if(_words_to_copy > 0){
				FlitType next = _ib->top(); _ib->pop();
				_mem->Write(_next_addr, (int8_t*)&next, 2);
				_next_addr += 2;
				_words_to_copy--;
			}else{
				_recv_state = NIRecvState::INTR_CPU;
			}
			break;
		
		case NIRecvState::INTR_CPU:
			_recv_intr->Write(1);		
			_recv_state = NIRecvState::IDLE;
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