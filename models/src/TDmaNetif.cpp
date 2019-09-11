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

#include <TDmaNetif.h>

//int xyz = 0;

TDmaNetif::TDmaNetif(std::string name) : TimedModel(name) {
      
    _signal_ack    = nullptr;
    _signal_intr   = nullptr;
    _signal_start  = nullptr;
    _signal_status = nullptr;
	    
    _ib = new UBuffer<FlitType>(name + ".IN", NI_BUFFER_LEN);
	
	this->Reset();
}

TDmaNetif::~TDmaNetif(){
    delete _ib;
}

void TDmaNetif::Reset(){
    	
	_recv_state = DmaNetifRecvState::READY;
	_flits_to_recv = 0;
	
    _send_state = DmaNetifSendState::READY;
	_flits_to_send = 0;
}

void TDmaNetif::SetOutputBuffer(UBuffer<FlitType>* ob){
	_ob = ob;
}

UBuffer<FlitType>* TDmaNetif::GetInputBuffer(){
	return _ib;
}

//state getters
DmaNetifRecvState TDmaNetif::GetRecvState(){
	return _recv_state;
}

DmaNetifSendState TDmaNetif::GetSendState(){
	return _send_state;
}

//recv mem
void TDmaNetif::SetMem1(UMemory* m1){ 
	_mem1 = m1; 
	_next_recv_addr = _mem1->GetBase();
}

//send mem
void TDmaNetif::SetMem2(UMemory* m2){
	_mem2 = m2; 
	_next_send_addr = _mem2->GetBase();
}

//signals
void TDmaNetif::SetSignalAck(USignal<int8_t>* c){ _signal_ack = c; }
void TDmaNetif::SetSignalIntr(USignal<int8_t>* c){ _signal_intr = c; }
void TDmaNetif::SetSignalStart(USignal<int8_t>* c){ _signal_start = c; }
void TDmaNetif::SetSignalStatus(USignal<int8_t>* c){ _signal_status = c; }

USignal<int8_t>* TDmaNetif::GetSignalAck(){ return _signal_ack; }
USignal<int8_t>* TDmaNetif::GetSignalIntr(){ return _signal_intr; }
USignal<int8_t>* TDmaNetif::GetSignalStart(){ return _signal_start; }
USignal<int8_t>* TDmaNetif::GetSignalStatus(){ return _signal_status; }

SimulationTime TDmaNetif::Run(){

	//independent processes, can run serial
    this->recvProcess();
    this->sendProcess();   
    
    return 1; //takes only 1 cycle to change both states
}

void TDmaNetif::recvProcess(){

	//recv state machine
	switch(_recv_state){
	
		//=============== STATE P0 => wait program for recv
		case DnRecvState::PROGRAM:{
		
			if(_recv_program->Read() == 1){
				_recv_address = _recv_address;
				_recv_state   = DmaNetifRecvState::READY;
			}
		
		}break;
	
		
		//=============== STATE S0 => wait for buffer activity
		case DmaNetifRecvState::READY:{
		
			//if  at least one packet is waiting,
			//pop that packet and wait for size flit
			if(_ib->size() > 0){
				
				_recv_addr_flit = _ib->top();
				_ib->pop();
				_recv_state = DmaNetifRecvState::LENGTH;			
			}
		}break;

		//=============== STATE S1 => wait for the size flit to arrive
		case DmaNetifRecvState::LENGTH:{
		
			//wait for the next flit
			if(_ib->size() > 0){

				_recv_size_flit = _ib->top();
				_ib->pop();
				
				//ask cpu to write to
				//_signal_recv_addr <= address to recv
				//_signal_recv_ok   <= flag to confirm addr
				_signal_intr->Write(INTR_RECV_ADDR_REQ);
				
				_recv_state = DmaNetifRecvState::WAITFLITS;
			}
		}break;
		
		//=============== STATE S2 => ask for address and wait remaining flits
		case DmaNetifRecvState::WAITFLITS:{
			
			//wait for the requested addr and remaining flits
			if(_ib->size() >= _recv_size_flit - 2){

				//stall cpu and start writing
				_signal_stall->Write(1);
				
				//set how many flit remains
				_recv_flits_to_copy = _recv_size_flit;
				
				_recv_state = DmaNetifRecvState::COPYHEADER;
			}
		}break;
		
		//=============== STATE S3 => copy header flit
		case DmaNetifRecvState::COPYHEADER:{
			//start writing to the first address
			_memory_slot_addr = _signal_recv_addr->Read();	
			_mem0->Write(_memory_slot_addr, (int8_t*)&_recv_addr_flit, sizeof(FlitType));
			
			//set next address (increment by one flit)
			_next_recv_addr += sizeof(FlitType);
			
			_recv_state = DmaNetifRecvState::COPYLENGTH;
		
		}break;
		
		//=============== STATE S4 => copy length flit
		case DmaNetifRecvState::COPYLENGTH:{

			//start writing to the first address
			_mem0->Write(_memory_slot_addr, (int8_t*)&_recv_length_flit, sizeof(FlitType));
			
			//set next address (increment by one flit)
			_memory_slot_addr += sizeof(FlitType);
			
			_recv_state = DmaNetifRecvState::COPYDATA;
		
		}break;
			
		//=============== STATE S5 => copy the rest of data
		case DmaNetifRecvState::COPYDATA:{

			if(_flits_to_recv == 0){
				_recv_state = DmaNetifRecvState::RELEASE;
				
			else{
				FlitType flit = _ib->top(); 
				_ib->pop();
			
				//start writing to the first address
				_mem0->Write(_memory_slot_addr, (int8_t*)&flit, sizeof(FlitType));
			
				//set next address (increment by one flit)
				_memory_slot_addr += sizeof(FlitType);
				_flits_to_recv -= 1;
			}
		}break;
		
		//=============== STATE S6 => release cpu and wait for next iteration	
		case DmaNetifRecvState::RELEASE:{
			
			_signal_stall->Write(0);
			_recv_state = DmaNetifRecvState::READY;
			
		}break;
	}
}

void TDmaNetif::sendProcess(){	
	//dummy
}
