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
#include <iomanip>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>

#include <TDmaNetif.h>

int _number_of_goddamn_packets = 0;

TDmaNetif::TDmaNetif(std::string name) : TimedModel(name) {
      
	_sig_stall = nullptr;
	_sig_intr  = nullptr;
	_sig_send_status = nullptr; 
	_sig_recv_status = nullptr; 
	_sig_prog_addr = nullptr; 
	_sig_prog_size = nullptr;
	_sig_prog_send = nullptr;
	_sig_prog_recv = nullptr;
	    
    _ib = new UBuffer<FlitType>(name + ".IN", BUFFER_CAPACITY);
	
	this->Reset();
}

TDmaNetif::~TDmaNetif(){
    delete _ib;
}

void TDmaNetif::Reset(){
    _recv_state = DmaNetifRecvState::WAIT_ADDR_FLIT;
    _send_state = DmaNetifSendState::WAIT_CONFIG_STALL;
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

//main mem
void TDmaNetif::SetMem0(UMemory* m0){ 
	_mem0 = m0; 
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
USignal<uint8_t>*  TDmaNetif::GetSignalStall(){ return _sig_stall; }
USignal<uint8_t>*  TDmaNetif::GetSignalIntr(){ return _sig_intr; }

USignal<uint8_t>*  TDmaNetif::GetSignalSendStatus(){ return _sig_send_status; }
USignal<uint32_t>*  TDmaNetif::GetSignalRecvStatus(){ return _sig_recv_status; }

USignal<uint8_t>*  TDmaNetif::GetSignalProgSend(){ return _sig_prog_send; }
USignal<uint8_t>*  TDmaNetif::GetSignalProgRecv(){ return _sig_prog_recv; }

USignal<uint32_t>* TDmaNetif::GetSignalProgAddr(){ return _sig_prog_addr; }
USignal<uint32_t>* TDmaNetif::GetSignalProgSize(){ return _sig_prog_size; }


//setters    
void TDmaNetif::SetSignalStall(USignal<uint8_t>* c){ _sig_stall = c; }
void TDmaNetif::SetSignalIntr(USignal<uint8_t>* c){ _sig_intr = c; }

void TDmaNetif::SetSignalSendStatus(USignal<uint8_t>* c){ _sig_send_status = c; }
void TDmaNetif::SetSignalRecvStatus(USignal<uint32_t>* c){ _sig_recv_status = c; }

void TDmaNetif::SetSignalProgSend(USignal<uint8_t>* c){ _sig_prog_send = c; }
void TDmaNetif::SetSignalProgRecv(USignal<uint8_t>* c){ _sig_prog_recv = c; }

void TDmaNetif::SetSignalProgAddr(USignal<uint32_t>* c){ _sig_prog_addr = c; }
void TDmaNetif::SetSignalProgSize(USignal<uint32_t>* c){ _sig_prog_size = c; }



SimulationTime TDmaNetif::Run(){

	//std::cout << this->GetName() << std::endl;

	//independent processes, can run serial
    this->recvProcess();
    this->sendProcess();
    
    return 1; //takes only 1 cycle to change both states
}


void TDmaNetif::recvProcess(){

	//if(this->GetName() == "004.netif" && _recv_payload_remaining != 0){
	//	std::cout << "recv remains: " << _recv_payload_remaining << std::endl;

	//}
	

	//recv state machine
	switch(_recv_state){

		//wait some flit to arrive at the local port
		case DmaNetifRecvState::WAIT_ADDR_FLIT:{
			
			//If buffer has any flit, copy that flit to internal 
			//memory and proceed to next flit. Please note that 
			//it is expected the first flit to containg address
			//data. Whatever comes first we treat as the addr flit.
			if(_ib->size() > 0){
			
				//copy the first flit into an auxiliar register and pop buffer
				_recv_reg = _ib->top(); 
				_ib->pop();
				
				//reset memory pointer and write copied flit to the first position
				_recv_address = 0;
				
				#ifdef NETIF_WRITE_ADDRESS_CHECKING
				if(_recv_address < _mem1->GetBase() || _recv_address > _mem1->GetLastAddr()){
					stringstream ss;
					ss << this->GetName() << ", recv::WAIT_ADDR_FLIT, unable to write to _mem1 " << std::hex << "0x" << _recv_address << std::endl;
					throw std::runtime_error(ss.str());
				}
				#endif 
				
				//write first flit to recv mem
				_mem1->Write(_recv_address, (int8_t*)&_recv_reg, sizeof(FlitType)); //write to mem
				_recv_address += sizeof(FlitType);
				
				//change states 
				_recv_state = DmaNetifRecvState::WAIT_SIZE_FLIT;
				
				//if(this->GetName() == "004.netif"){
				//	std::cout << "recv addr 0x" << std::fixed << setfill('0') << setw(4) << std::hex << _recv_reg << std::endl;
				
				//}
			}
		} break;
		
		//read size flit to determine how many flits will come next. please
		//note that we treat whatever flit comes next as the size flit
		case DmaNetifRecvState::WAIT_SIZE_FLIT:{
		
			if(_ib->size() > 0){
			
				//copy size flit to an auxiliar register and pop buffer
				_recv_reg = _ib->top();
				_ib->pop();

				//report current size (in flits) to the cpu (we sum 2 due to the size flit
				//does not take the first two flits into account. 
				_sig_recv_status->Write((_recv_reg + 2));
				
				#ifdef NETIF_WRITE_ADDRESS_CHECKING
				if(_recv_address < _mem1->GetBase() || _recv_address > _mem1->GetLastAddr()){
					stringstream ss;
					ss << this->GetName() << ", recv::WAIT_SIZE_FLIT, unable to write to _mem1 " << std::hex << "0x" << _recv_address << std::endl;
					throw std::runtime_error(ss.str());
				}
				#endif 
				
				//write to the second position and increment memory pointer
				_mem1->Write(_recv_address, (int8_t*)&_recv_reg, sizeof(FlitType));
				_recv_address += sizeof(FlitType);
				
				//set current payload size and the number of remaining flits
				_recv_payload_size = _recv_reg;
				_recv_payload_remaining = _recv_reg;
				
				//change states
				_recv_state = DmaNetifRecvState::WAIT_PAYLOAD;
				
				//if(this->GetName() == "003.netif")
				//	std::cout << "recv size 0x" << std::fixed << setfill('0') << setw(4) << std::hex << _recv_reg << std::endl;
			}

		} break;
		
		//wait for remaining flits to arrive, and interrupt
		case DmaNetifRecvState::WAIT_PAYLOAD:{

			//check whether there are more flits to receive
			if(_recv_payload_remaining > 0){
			
				//check whether there is any data in the buffer
				if(_ib->size() > 0){
			
					//copy one flit into the auxiliary register and pop buffer
					_recv_reg = _ib->top();
					_ib->pop();

					#ifdef NETIF_WRITE_ADDRESS_CHECKING
					if(_recv_address < _mem1->GetBase() || _recv_address > _mem1->GetLastAddr()){
						stringstream ss;
						ss << this->GetName() << ", recv::WAIT_PAYLOAD, unable to write to _mem1 " << std::hex << "0x" << _recv_address << std::endl;
						throw std::runtime_error(ss.str());
					}
					#endif 

					//write the flit to memory and increment memory counter
					_mem1->Write(_recv_address, (int8_t*)&_recv_reg, sizeof(FlitType));
					_recv_address += sizeof(FlitType);
					
					//one less flit to be received
					_recv_payload_remaining--;
					
					//if(this->GetName() == "003.netif")
					//	std::cout << "recv data 0x" << std::fixed << setfill('0') << setw(4) << std::hex << _recv_reg << ", " << _recv_payload_remaining << " remaining" << std::endl;
				}

			//whether the ni received all the payload, interrupt the cpu and change states
			}else{
				_sig_intr->Write(0x1);
				_recv_state = DmaNetifRecvState::WAIT_CONFIG_STALL;
			}
		} break;

		//wait for the cpu to configure the dma
		case DmaNetifRecvState::WAIT_CONFIG_STALL:{
		
			//if the prog_recv signal has been set, stall the cpu,
			//configure the number of flits to copy to the main memory
			//and then chagne states
			//if(this->GetName() == "004.netif")
			//	std::cout << "0x" << std::hex << (int)_sig_prog_recv->Read() << std::endl;
			
			if(_sig_prog_recv->Read() == 0x1){
			
				//std::stringstream sname;
				//sname << "breakpoints/packet" << _number_of_goddamn_packets++ << ".bin";
				
				//_mem1->SaveBin(sname.str(), 0, _mem1->GetSize());
				
			
				//printf("NI ADDR: 0x%x\n", _sig_prog_addr->Read());
			
				//configured via cpu. accounts the +2, so no 
				//sum is necessary
				_recv_payload_remaining = _sig_prog_size->Read(); 
				_sig_recv_status->Write(0x0);
				
				_recv_state = DmaNetifRecvState::COPY_RELEASE;
				_recv_address = 0; //reset memory pointer

				_sig_stall->Write(0x1); //stall cpu
			}

		} break;

		//copy data, and release
		case DmaNetifRecvState::COPY_RELEASE:{
		
			//for each flit, copy from the auxiliary _mem1 memory
			//to the _mem0 main memory
			if(_recv_payload_remaining > 0){

				#ifdef NETIF_READ_ADDRESS_CHECKING
				if(_recv_address < _mem1->GetBase() || _recv_address > _mem1->GetLastAddr()){
					stringstream ss;
					ss << this->GetName() << ", recv::COPY_RELEASE, unable to read to _mem1 " << std::hex << "0x" << _recv_address << std::endl;
					throw std::runtime_error(ss.str());
				}
				#endif 

				//read data to the auxiliary register
				_mem1->Read(_recv_address, (int8_t*)&_recv_reg, sizeof(FlitType));
				
				#ifdef NETIF_WRITE_ADDRESS_CHECKING
				uint32_t addr = _recv_address + _sig_prog_addr->Read();
				if(addr < _mem0->GetBase() || addr > _mem0->GetLastAddr()){
					
					stringstream ss;
					ss << this->GetName() << ", recv::COPY_RELEASE, unable to write to _mem0 " << std::hex << "0x" << addr << std::endl;
					throw std::runtime_error(ss.str());
				}
				#endif 

				//write auxiliary flit to main memory
				_mem0->Write(_recv_address + _sig_prog_addr->Read(), (int8_t*)&_recv_reg, sizeof(FlitType));
				
				_recv_address += sizeof(FlitType); //read next address
				_recv_payload_remaining--; //one less flit to write to the memory
				
				//if(this->GetName() == "003.netif")
				//	std::cout << "recv copy 0x" << std::fixed << setfill('0') << setw(4) << std::hex << _recv_reg << ", " << _recv_payload_remaining << " remaining" << std::endl;
				
			//if there is no more flits to receive, lower the interruption,
			//restore the cpu (release stall), then change states
			}else{

				_sig_stall->Write(0x0);
				_sig_intr->Write(0x0);

				_recv_state = DmaNetifRecvState::FLUSH;

				// if(this->GetName() == "003.netif")
				// 	std::cout << "recv end" << std::endl;
			}
		} break;
				
		case DmaNetifRecvState::FLUSH:{	

			//if(this->GetName() == "004.netif")
			//	std::cout << "flushed: " << xx++ << std::endl;

			//return to the main state on the acknowledge
			// --- wait for the buffer to get full to interrupt again
			if(_sig_prog_recv->Read() == 0x0){// && _ib->size() > 0){
				
				_recv_state = DmaNetifRecvState::WAIT_ADDR_FLIT;
				
				//if(this->GetName() == "003.netif")
				//	std::cout << "flushed" << std::endl;
				
				//if(this->GetName() == "004.netif")
				//	xx = 0;
			}
		} break;
	}
}

void TDmaNetif::sendProcess(){

	//send state machine
	switch(_send_state){

		//wait the cpu to configure the ni
		case DmaNetifSendState::WAIT_CONFIG_STALL:{
			
			if(_sig_prog_send->Read() == 0x1){
				
				_sig_stall->Write(0x1);        //raise stall
				_sig_send_status->Write(0x1);  //raise status

				_send_address = 0;
				_send_payload_size = _sig_prog_size->Read();
				_send_payload_remaining = _send_payload_size;
				
				_send_state = DmaNetifSendState::COPY_AND_RELEASE; //change states
				
				//std::cout << "send started" << std::endl;
			}
			
		} break;

		//copy data from the main memory to the auxiliary memory
		//and releases cpu, lower status and stall
		case DmaNetifSendState::COPY_AND_RELEASE:{
			
			if(_send_payload_remaining > 0){
				
				#ifdef NETIF_READ_ADDRESS_CHECKING
				uint32_t addr = _send_address + _sig_prog_addr->Read();
				if(addr < _mem0->GetBase() || addr > _mem0->GetLastAddr()){
					
					stringstream ss;
					ss << this->GetName() << ", send::COPY_RELEASE, unable to read from _mem0 " << std::hex << "0x" << addr << std::endl;
					throw std::runtime_error(ss.str());
				}
				#endif 
								
				//read from main memory
				_mem0->Read(_send_address + _sig_prog_addr->Read(), (int8_t*)&_send_reg, sizeof(FlitType));
				
				#ifdef NETIF_WRITE_ADDRESS_CHECKING
				if(_send_address < _mem2->GetBase() || _send_address > _mem2->GetLastAddr()){
					
					stringstream ss;
					ss << this->GetName() << ", send::COPY_RELEASE, unable to write to _mem2 " << std::hex << "0x" << _send_address << std::endl;
					throw std::runtime_error(ss.str());
				}
				#endif 
				
				//write auxiliary flit to auxiliary memory
				_mem2->Write(_send_address, (int8_t*)&_send_reg, sizeof(FlitType));
				
				_send_address += sizeof(FlitType); //write next address
				_send_payload_remaining--; //one less packet to send
				
				// std::cout << "send copied 0x" << std::fixed << setfill('0') << setw(4) << std::hex << _send_reg << std::endl;

			//all flits copied to the aux memory, switch to noc-mode
			}else{
				
				// std::cout << "send stalled" << std::endl;
							
				_sig_stall->Write(0x0);        //low stall
				_send_payload_remaining = _send_payload_size; //flits to push to the noc
				_send_address = 0;  //reset memory pointer

				_send_state = DmaNetifSendState::SEND_DATA_TO_NOC;
			}

		} break;	
		
		case DmaNetifSendState::SEND_DATA_TO_NOC:{
	
			//make sure the buffer has room to receive another packet
			if(_send_payload_remaining > 0){

				if(_ob->size() < _ob->capacity()){
				
					_mem2->Read(_send_address, (int8_t*)(&_send_reg), sizeof(FlitType));
					_ob->push(_send_reg);
					_send_payload_remaining--;
					
					_send_address += sizeof(FlitType);

					//std::cout << "send pushed 0x" << std::fixed << setfill('0') << setw(4) << std::hex << _send_reg << ", " << _send_payload_remaining << " remaining" <<std::endl;

				}

			//make sure the cpu have lowered the start signal at least once before 
			//getting back to the waiting state (prevent duplicates)
			}else {
				_send_state = DmaNetifSendState::FLUSH;
			}
		} break;

		case DmaNetifSendState::FLUSH:{	

			if(_sig_prog_send->Read() == 0x0){
				
				_sig_send_status->Write(0x0);  //notify free
				_send_state = DmaNetifSendState::WAIT_CONFIG_STALL;
				
				//if(this->GetName() == "003.netif")
				//	std::cout << "flushed" << std::endl;				
			}
		} break;
	}
}
