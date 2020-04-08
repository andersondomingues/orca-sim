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
	USignal<uint8_t>* stall, USignal<uint8_t>* dma_start, USignal<uint32_t>* burst_size, USignal<uint32_t>* weight_mem_addr, 
	USignal<uint32_t>* input_mem_addr, USignal<float>* mac_out,
	uint32_t memW, uint32_t memI, uint32_t mem_height, UMemory* main_mem, TimedFPMultiplier* mac
	) : TimedModel(name) {
      
	// control signal sent to the proc
	_sig_stall = stall;
	_sig_dma_prog = dma_start;

	// data signals sent by the proc
	_sig_burst_size = burst_size;
	_sig_weight_mem_addr = weight_mem_addr;
	_sig_input_mem_addr = input_mem_addr;
	// data signals sent to the proc
	_sig_mac_out = mac_out;
	// constants
	_mem_height = mem_height;
	_memW = memW;
	_memI = memI;

	_mem0 = main_mem;
	
	// internal control 'registers' between the pipeline stages
	_mul_loaded  = 0;
	_mul_ready   = 0;

	// internal data 'registers' between the pipeline stages
	_reg_mul = 0;
	_reg_mac = 0;

	// binding the modules
    _mult = mac;

	this->Reset();
}

TDmaMult::~TDmaMult(){
}

void TDmaMult::Reset(){
    // all relevant data go to their initial value at this state
	_dma_state = DmaState::WAIT_CONFIG_STALL;
	_sig_mac_out->Write(0.0f);
}

DmaState TDmaMult::GetDmaState(){
	return _dma_state;
}

//getters
/*
USignal<uint8_t>*  TDmaMult::GetSignalStall(){ return _sig_stall; }
USignal<uint8_t>*  TDmaMult::GetSignalDmaProg(){ return _sig_dma_prog; }

USignal<uint32_t>* TDmaMult::GetSignalBurstSize(){ return _sig_burst_size; }
USignal<uint32_t>* TDmaMult::GetSignalWeightMemAddr(){ return _sig_weight_mem_addr; }
USignal<uint32_t>* TDmaMult::GetSignalInputMemAddr(){ return _sig_input_mem_addr; }
USignal<uint32_t>* TDmaMult::GetSignalMacOut(){ return _sig_mac_out; }
*/

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch"
void TDmaMult::DoAcc(){
	switch(_dma_state){	
		case DmaState::WAIT_CONFIG_STALL:{
			_reg_mac = 0;
			}break;
		case DmaState::COPY_FROM_MEM:{
			if (_mul_ready == 0x1){
				_reg_mac += _reg_mul;
			} 
			}break;
		case DmaState::COPY_TO_CPU:{
/*			
			union {
				float f;
				uint32_t u;
				uint8_t b[4];
			} f2u ;
			int8_t *ptr;
			//float *fptr;

			ptr = _mem0->GetMap(0x4041200C);
			//fptr = (float *)ptr;

			f2u.b[0] = *ptr;
			f2u.b[1] = *(ptr+1);
			f2u.b[2] = *(ptr+2);
			f2u.b[3] = *(ptr+3);

			printf ("MAC0 : %f - 0x%0x\n", f2u.f, f2u.u);
			printf("MAC0 char : ");
			for(uint8_t i=0;i<4;i++){
				printf(" %02hhx,", *ptr++);
			}
			printf("\n");
*/
			uint32_t * ptr = (uint32_t *)&_reg_mac;
			_sig_mac_out->Write(_reg_mac);  // send the final result back to the processor
			printf ("MAC : %f - 0x%0x\n", _reg_mac, *ptr);
			//*fptr = _reg_mac;
			//printf("MAC  %f\n", _reg_mac);
/*
			f2u.f = _reg_mac;

			printf ("MAC : %f\n   : 0x%0x\n", _reg_mac, f2u.u);
			char *pt = (char *)&_reg_mac;
			printf("MAC char : ");
			for(uint8_t i=0;i<sizeof(_reg_mac);i++){
				printf(" %02hhx,", *pt++);
			}
			printf("\n");


			ptr = _mem0->GetMap(0x4041200C);

			f2u.b[0] = *ptr;
			f2u.b[1] = *(ptr+1);
			f2u.b[2] = *(ptr+2);
			f2u.b[3] = *(ptr+3);

			printf ("MAC2 : %f - 0x%0x\n", f2u.f, f2u.u);
			printf("MAC2 char : ");
			for(uint8_t i=0;i<4;i++){
				printf(" %02hhx,", *ptr++);
			}
			printf("\n");

*/
			//printf("\nMAC int : %x\n", *(unsigned int*)&_reg_mac);
			}break;
		case DmaState::FLUSH:
			break;
	}
}
#pragma GCC diagnostic pop

// 2rd pipeline stage - do the mult
void TDmaMult::DoMult(){
	if (_dma_state == DmaState::WAIT_CONFIG_STALL){
		_reg_mul = 0; // restart register 
		_mul_ready = 0;
	}
	else {
		if (_mul_loaded == 0x1){
			_reg_mul = _mult->GetResult();
			printf("MULT  %f\n", _reg_mul);
			_mul_ready = 1;
		}else{
			_mul_ready = 0;
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
				_mul_loaded = 0;   // raised when the mul can be executed
				
				// the processor must write: 
				// 	  - the size of the burst (uint32_t), 
				//    - the inital address of the weight memory, 
				//    - the inital address of the input memory, 
				// output: _sig_size + 2 cycles latter, the register _reg_mac has the result
				_burst_size = _sig_burst_size->Read();
				if (_burst_size >_mem_height){
					stringstream s;
					s << this->GetName() << ": burst size exedded the NN memory capacity.";
					throw std::runtime_error(s.str());
				} 
				//_weight_mem_addr = _sig_weight_mem_addr->Read();
				//_input_mem_addr = _sig_input_mem_addr->Read();
				_w_mem_idx = _memW;
				_i_mem_idx = _memI;
				_remaining = _burst_size;
				// TODO one alternative is to receive only the burst size from the processor and have the base addr fixed
				//_weight_mem_addr = _memW->GetAddress();
				//_input_mem_addr  = _memI->GetAddress();
				
				_dma_state = DmaState::COPY_FROM_MEM; //change states
				
				//std::cout << "send started" << std::endl;
			}
			
		} break;

		//copy data from the main memory to the auxiliary memory
		//and releases cpu, lower status and stall
		case DmaState::COPY_FROM_MEM:{
			
			if(_remaining > 0){
				// there are not registers, but simple wires connecting the mem to the mult
				uint32_t input_wire;
				int8_t * w_ptr, * i_ptr;
				float weight_wire, input_wire_f;
				
				// #ifdef NETIF_READ_ADDRESS_CHECKING
				// uint32_t addr = _send_address + _sig_prog_addr->Read();
				// if(addr < _mem0->GetBase() || addr > _mem0->GetLastAddr()){
					
				// 	stringstream ss;
				// 	ss << this->GetName() << ", send::COPY_RELEASE, unable to read from _mem0 " << std::hex << "0x" << addr << std::endl;
				// 	throw std::runtime_error(ss.str());
				// }
				// #endif 
								
				//read from main memory
//				_memW->Read(_weight_mem_addr , (int8_t*)&weight_wire, sizeof(uint32_t));
//				_memI->Read(_input_mem_addr , (int8_t*)&input_wire, sizeof(uint32_t));
				
				w_ptr = _mem0->GetMap(_w_mem_idx);
				weight_wire = *(float*)w_ptr;
				i_ptr = _mem0->GetMap(_i_mem_idx);
				input_wire  = *(uint32_t*)i_ptr;
				input_wire_f = (float)input_wire;

				// #ifdef NETIF_WRITE_ADDRESS_CHECKING
				// if(_send_address < _mem2->GetBase() || _send_address > _mem2->GetLastAddr()){
					
				// 	stringstream ss;
				// 	ss << this->GetName() << ", send::COPY_RELEASE, unable to write to _mem2 " << std::hex << "0x" << _send_address << std::endl;
				// 	throw std::runtime_error(ss.str());
				// }
				// #endif 

				//printf("READING W (0x%X - %p): %f\n", _w_mem_idx, w_ptr, weight_wire);
				//printf("READING I (0x%X - %p): %f\n", _i_mem_idx, i_ptr, input_wire_f);
				
				//write auxiliary flit to auxiliary memory
				_mult->SetOp1(weight_wire);
				_mult->SetOp2(input_wire_f);
				_mul_loaded = 1; 
				
				_remaining--; //one less packet to send
				_w_mem_idx +=4;
				_i_mem_idx +=4;
				//_weight_mem_addr++;
				//_input_mem_addr++;
				
				// std::cout << "send copied 0x" << std::fixed << setfill('0') << setw(4) << std::hex << _send_reg << std::endl;

			//all flits copied to the aux memory, switch to noc-mode
			}else{
				
				// std::cout << "send stalled" << std::endl;
				_mul_loaded = 0; 
				_dma_state = DmaState::COPY_TO_CPU;
			}

		} break;	

		case DmaState::COPY_TO_CPU:{
			// result is written back to the output MMIO register
			_dma_state = DmaState::FLUSH;
			}break;

		// just waits few clock cycles. currently, only one cycle
		case DmaState::FLUSH:
			float res;
			uint32_t iptr;
			int8_t * bptr;
			// reading from DMA_MAC_OUT addr
			// convert int8 pointer to float pointer, then get the data
			res = *((float*)_mem0->GetMap(0x4041200C));
			iptr = *((uint32_t*)_mem0->GetMap(0x4041200C));
			bptr = _mem0->GetMap(0x4041200C);

			// print the host addr, the data in float, in hex, and bytes
			printf ("MAC3 (%p): %f - 0x%0x\n", bptr, res, iptr);
			printf("MAC3 char : ");
			for(uint8_t i=0;i<4;i++){
				printf(" %02hhx,", *bptr++);
			}
			printf("\n");

			_dma_state = DmaState::WAIT_CONFIG_STALL;
			_sig_dma_prog->Write(0x0);     //lower the start signal
			_sig_stall->Write(0x0);        //lowering stall and giving the control back to the processor
			break;
	}
}
