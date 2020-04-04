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

#include <TDmaMult.h>

//int _number_of_goddamn_packets = 0;

TDmaMult::TDmaMult(std::string name,  
	//signals
	USignal<uint8_t>* stall, USignal<uint32_t>* burst_size, USignal<uint32_t>* weight_mem_addr, 
	USignal<uint32_t>* input_mem_addr, USignal<uint32_t>* mac_out,
	//modules
	UMemory* memW, UMemory* memI, TimedFPMultiplier* mac
	) : TimedModel(name) {
      
	// control signal sent to the proc
	_sig_stall = stall;

	// data signals sent by the proc
	_sig_burst_size = burst_size;
	_sig_weight_mem_addr = weight_mem_addr;
	_sig_input_mem_addr = input_mem_addr;
	// data signals sent to the proc
	_sig_mac_out = mac_out;
	
	// internal control 'registers' between the pipeline stages
	_mul_loaded  = 0;
	_mul_ready   = 0;

	// internal data 'registers' between the pipeline stages
	_reg_mul = 0;
	_reg_mac = 0;

	// binding the modules
    _mult = mac;
	_memW = memW;
	_memI = memI;
	
	this->Reset();
}

TDmaMult::~TDmaMult(){
}

void TDmaMult::Reset(){
    // all relevant data go to their initial value at this state
	_dma_state = DmaState::WAIT_CONFIG_STALL;
}

DmaState TDmaMult::GetDmaState(){
	return _dma_state;
}

//weight memory
void TDmaMult::SetMemW(UMemory* m0){ 
	_memW = m0; 
}

// input memory
void TDmaMult::SetMemI(UMemory* m0){ 
	_memI = m0; 
}

//Multiplier
void TDmaMult::SetMult(TimedFPMultiplier* mul){ 
	_mult = mul; 
}

//getters
USignal<uint8_t>*  TDmaMult::GetSignalStall(){ return _sig_stall; }
USignal<uint8_t>*  TDmaMult::GetSignalIntr(){ return _sig_intr; }
USignal<uint8_t>*  TDmaMult::GetSignalDmaStatus(){ return _sig_dma_status; }
USignal<uint8_t>*  TDmaMult::GetSignalDmaProg(){ return _sig_dma_prog; }

USignal<uint32_t>* TDmaMult::GetSignalBurstSize(){ return _sig_burst_size; }
USignal<uint32_t>* TDmaMult::GetSignalWeightMemAddr(){ return _sig_weight_mem_addr; }
USignal<uint32_t>* TDmaMult::GetSignalInputMemAddr(){ return _sig_input_mem_addr; }
USignal<uint32_t>* TDmaMult::GetSignalMacOut(){ return _sig_mac_out; }

//setters    
void TDmaMult::SetSignalStall(USignal<uint8_t>* c){ _sig_stall = c; }
void TDmaMult::SetSignalIntr(USignal<uint8_t>* c){ _sig_intr = c; }
void TDmaMult::SetSignalDmaStatus(USignal<uint8_t>* c){ _sig_dma_status = c; }
void TDmaMult::SetSignalDmaProg(USignal<uint8_t>* c){ _sig_dma_prog = c; }

void TDmaMult::SetSignalBurstSize(USignal<uint32_t>* c){ _sig_burst_size = c; }
void TDmaMult::SetSignalWeightMemAddr(USignal<uint32_t>* c){ _sig_weight_mem_addr = c; }
void TDmaMult::SetSignalInputMemAddr(USignal<uint32_t>* c){ _sig_input_mem_addr = c; }
void TDmaMult::SetSignalMacOut(USignal<uint32_t>* c){ _sig_mac_out = c; }


SimulationTime TDmaMult::Run(){

	//std::cout << this->GetName() << std::endl;
	// TODO fazer loop p realizar o mac de todo o vetor
	//3 stage pipeline
	this->DoAcc();     // 3rd stage, accumulator
	this->DoMult();    // 2nd stage, does mult
	this->ReadData();  // 1st stage, read data from the memory

    return 1; //takes only 1 cycle to change both states
}

// 3rd pipeline stage - accumulate previous value with current mult result
void TDmaMult::DoAcc(){
	switch(_dma_state){	
		case DmaState::WAIT_CONFIG_STALL:
			_reg_mac->Write(0x0);
			_sig_mac_out->Write(0x0);
			break;
		case DmaState::COPY_FROM_MEM:
			if (_sig_mul_ready->Read() == 0x1){
				_reg_mac->Write(_reg_mac->Read() + _reg_mul->Read());
			} break;
		case DmaState::FLUSH:
			_sig_mac_out->Write(_reg_mac->Read());  // send the final result back to the processor
			//TODO missing some control signal here
			// liberar o stall
			break;
	}
}

// 2rd pipeline stage - do the mult
void TDmaMult::DoMult(){
	if (_dma_state == DmaState::WAIT_CONFIG_STALL){
		_reg_mul->Write(0x0); // restart register 
	}
	else {
		if (_sig_mul_loaded->Read() == 0x1){
			_reg_mul->Write(_mult->GetResult());
			_sig_mul_ready->Write(0x1);
		}else{
			_sig_mul_ready->Write(0x0);
		}
	}
}

// 1st pipeline stage - read the memories and load the mult operands
void TDmaMult::ReadData(){

	//send state machine
	switch(_dma_state){

		//wait the cpu to configure the ni
		case DmaState::WAIT_CONFIG_STALL:{
			
			if(_sig_dma_prog->Read() == 0x1){
				
				_sig_stall->Write(0x1);        //raise stall
				_sig_dma_status->Write(0x1);   //raise status

				_sig_mul_loaded->Write(0x0);   // raised when the mul can be executed
				
				// the processor must write: 
				// 	  - the size of the burst (uint32_t), 
				//    - the inital address of the weight memory, 
				//    - the inital address of the input memory, 
				// output: _sig_size + 2 cycles latter, the register _reg_mac has the result
				_burst_size = _sig_burst_size->Read();
				_weight_mem_addr = _sig_weight_mem_addr->Read();
				_input_mem_addr = _sig_input_mem_addr->Read();
				_remaining = _burst_size;
				
				_dma_state = DmaState::COPY_FROM_MEM; //change states
				
				//std::cout << "send started" << std::endl;
			}
			
		} break;

		//copy data from the main memory to the auxiliary memory
		//and releases cpu, lower status and stall
		case DmaState::COPY_FROM_MEM:{
			
			if(_remaining > 0){
				// there are not registers, but simple wires connecting the mem to the mult
				uint32_t weight_wire, input_wire;  
				
				// #ifdef NETIF_READ_ADDRESS_CHECKING
				// uint32_t addr = _send_address + _sig_prog_addr->Read();
				// if(addr < _mem0->GetBase() || addr > _mem0->GetLastAddr()){
					
				// 	stringstream ss;
				// 	ss << this->GetName() << ", send::COPY_RELEASE, unable to read from _mem0 " << std::hex << "0x" << addr << std::endl;
				// 	throw std::runtime_error(ss.str());
				// }
				// #endif 
								
				//read from main memory
				_memW->Read(_weight_mem_addr , (int8_t*)&weight_wire, sizeof(uint32_t));
				_memI->Read(_input_mem_addr , (int8_t*)&input_wire, sizeof(uint32_t));
				
				// #ifdef NETIF_WRITE_ADDRESS_CHECKING
				// if(_send_address < _mem2->GetBase() || _send_address > _mem2->GetLastAddr()){
					
				// 	stringstream ss;
				// 	ss << this->GetName() << ", send::COPY_RELEASE, unable to write to _mem2 " << std::hex << "0x" << _send_address << std::endl;
				// 	throw std::runtime_error(ss.str());
				// }
				// #endif 
				
				//write auxiliary flit to auxiliary memory
				_mult->SetOp1(weight_wire);
				_mult->SetOp2(input_wire);
				_sig_mul_loaded->Write(0x1); 
				
				_remaining--; //one less packet to send
				_weight_mem_addr++;
				_input_mem_addr++;
				
				// std::cout << "send copied 0x" << std::fixed << setfill('0') << setw(4) << std::hex << _send_reg << std::endl;

			//all flits copied to the aux memory, switch to noc-mode
			}else{
				
				// std::cout << "send stalled" << std::endl;
							
				_sig_stall->Write(0x0);        //low stall
				_sig_mul_loaded->Write(0x0); 
				_dma_state = DmaState::FLUSH;
			}

		} break;	

		case DmaState::FLUSH:{	

			_sig_mul_loaded->Write(0x0);
			if(_sig_dma_prog->Read() == 0x0){
				
				_sig_dma_status->Write(0x0);  //notify free
				_dma_state = DmaState::WAIT_CONFIG_STALL;			
			}
		} break;
	}
}
