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
      
    _sig_stall = nullptr;
	_sig_intr  = nullptr;
	_sig_send_status = nullptr; 
	_sig_recv_status = nullptr; 
	_sig_prog_addr = nullptr; 
	_sig_prog_size = nullptr;
	_sig_prog_send = nullptr;
	_sig_prog_recv = nullptr;
	    
    _ib = new UBuffer<FlitType>(name + ".IN", NI_BUFFER_LEN);
	
	this->Reset();
}

TDmaNetif::~TDmaNetif(){
    delete _ib;
}

void TDmaNetif::Reset(){
    _recv_state = DmaNetifRecvState::WAIT_ADDR_FLIT;
    _send_state = DmaNetifSendState::READY;
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
}

//send mem
void TDmaNetif::SetMem2(UMemory* m2){
	_mem2 = m2; 
}

//getters
USignal<int8_t>*  TDmaNetif::GetSigStall(){ return _sig_stall; }
USignal<int8_t>*  TDmaNetif::GetSigIntr(){ return _sig_intr; }
USignal<int8_t>*  TDmaNetif::GetSigSendStatus(){ return _sig_send_status; }
USignal<int8_t>*  TDmaNetif::GetSigRecvStatus(){ return _sig_recv_status; }
USignal<int32_t>* TDmaNetif::GetSigProgAddr(){ return _sig_prog_addr; }
USignal<int32_t>* TDmaNetif::GetSigProgSize(){ return _sig_prog_size; }
USignal<int8_t>*  TDmaNetif::GetSigProgSend(){ return _sig_prog_send; }
USignal<int8_t>*  TDmaNetif::GetSigProgRecv(){ return _sig_prog_recv; }

//setters    
void TDmaNetif::SetSigStall(USignal<int8_t>* c){ _sig_stall = c; }
void TDmaNetif::SetSigIntr(USignal<int8_t>* c){ _sig_intr = c; }
void TDmaNetif::SetSigSendStatus(USignal<int8_t>* c){ _sig_send_status = c; }
void TDmaNetif::SetSigRecvStatus(USignal<int8_t>* c){ _sig_recv_status = c; }
void TDmaNetif::SetSigProgAddr(USignal<int32_t>* c){ _sig_prog_addr = c; }
void TDmaNetif::SetSigProgSize(USignal<int32_t>* c){ _sig_prog_size = c; }
void TDmaNetif::SetSigProgSend(USignal<int8_t>* c){ _sig_prog_send = c; }
void TDmaNetif::SetSigProgRecv(USignal<int8_t>* c){ _sig_prog_recv = c; }

SimulationTime TDmaNetif::Run(){

	//independent processes, can run serial
    this->recvProcess();
    this->sendProcess();   
    
    return 1; //takes only 1 cycle to change both states
}

void TDmaNetif::recvProcess(){

	//recv state machine
	switch(_recv_state){

		//wait some flit to arrive at the local port
		case DmaNetifRecvState::WAIT_ADDR_FLIT:{
			
			//If buffer has any flit, copy that flit to internal 
			//memory and proceed to next flit. Please note that 
			//it is expected the first flit to containg address
			//data. Whatever comes first we treat as the addr flit.
			if(_ib->size() > 0){
				_recv_reg = _ib->top(); //remove flit from buffer
				_ib->pop();
						
				_recv_address = 0;
				_mem1->Write(_recv_address++ * sizeof(FlitType),
					 (int8_t*)&_recv_reg, sizeof(FlitType)); //write to mem

				_sig_recv_status->Write(0x1);
				
				_recv_state = DmaNetifRecvState::WAIT_SIZE_FLIT;
			}
		} break;
		
		//read size flit to determine how many flits will come next
		case DmaNetifRecvState::WAIT_SIZE_FLIT:{
		
			if(_ib->size() > 0){
				_recv_reg = _ib->top();
				_ib->pop();
				
				//write to the second position
				_mem1->Write(_recv_address++ * sizeof(FlitType),
					 (int8_t*)&_recv_reg, sizeof(FlitType));
				
				_recv_payload_size = _recv_reg;
				_recv_payload_remaining = _recv_reg;
				
				_recv_state = DmaNetifRecvState::WAIT_PAYLOAD;
			}
		
		} break;
		
		//wait for remaining flits to arrive, and interrupt
		case DmaNetifRecvState::WAIT_PAYLOAD:{
		
			//check whether more flits are coming
			if(_ib->size() > 0){	
				_recv_reg = _ib->top();
				_ib->pop();

				_mem1->Write(_recv_address++ * sizeof(FlitType),
					(int8_t*)&_recv_reg, sizeof(FlitType));
					
				_recv_payload_remaining--;
				
				if(_recv_payload_remaining == 0){
					_sig_intr->Write(0x1);
					_recv_state = DmaNetifRecvState::WAIT_CONFIG_STALL;
				}
			}
		} break;

		//wait for the cpu to configure the dma
		case DmaNetifRecvState::WAIT_CONFIG_STALL:{
		
			//start is the last wire to be set by the driver
			if(_sig_prog_recv->Read() == 0x1){
				_sig_stall->Write(0x1); //stall cpu
				_recv_payload_remaining = _recv_payload_size;

				_recv_state = DmaNetifRecvState::COPY_RELEASE;
			}
		} break;
		
		//stalls cpu, copy data, and release
		case DmaNetifRecvState::COPY_RELEASE:{
		
			if(_recv_payload_remaining > 0){
				//calcular endereço
				uint32_t addr = 0x0;
				_mem1->Read(addr, (int8_t*)&_recv_reg, sizeof(FlitType));
				_mem0->Write(addr, (int8_t*)&_recv_reg, sizeof(FlitType));
				_recv_payload_remaining--;
				
			}else{
				//restore cpu 
				_sig_stall->Write(0x0);
				_recv_state = DmaNetifRecvState::WAIT_ACK;
			}
		
		} break;

		//wait for cpu acknowledgement
		case DmaNetifRecvState::WAIT_ACK:{
				
			if(_sig_prog_send->Read() == 0x0){
				_sig_recv_status->Write(0x0);
				_recv_state = DmaNetifRecvState::WAIT_ADDR_FLIT;
			}
		
		} break;
	}
}

void TDmaNetif::sendProcess(){	
	//dummy
}
