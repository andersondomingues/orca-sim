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

TDmaMult::TDmaMult(std::string name,  
	//signals
	USignal<uint8_t>* stall, USignal<uint8_t>* dma_start, USignal<uint32_t>* burst_size, USignal<uint32_t>* nn_size, 
	USignal<uint32_t>* out_size, uint32_t base_mac_out_addr, UMemory* main_mem) : TimedModel(name) {
    
	int i;

	// control signal sent to the proc
	_sig_stall = stall;
	_sig_dma_prog = dma_start;

	// data signals sent by the proc
	_sig_burst_size = burst_size;
	_sig_nn_size = nn_size;   // TODO not used
	_sig_out_size = out_size; // TODO not used
	_base_mac_out_addr = base_mac_out_addr;

	// set the base mamory address to each channel
	_memW[0] = MEMW_BASE;
	_memI[0] = MEMI_BASE;
	for ( i = 1; i < SIMD_SIZE; i++)
	{
		_memW[i] = _memW[i-1] + NN_MEM_SIZE_PER_CHANNEL;
		_memI[i] = _memI[i-1] + NN_MEM_SIZE_PER_CHANNEL;
	}
	_mem0 = main_mem;
	
	// internal control 'registers' between the pipeline stages
	_mul_loaded  = 0;
	_mul_ready   = 0;

	printf("NN CONFIGURATION:\n\n");
	printf("  TOTAL_NN_MEM_SIZE = 0x%x\n",TOTAL_NN_MEM_SIZE);
	printf("  SIMD_SIZE = %d\n",SIMD_SIZE);
	printf("  NN_MEM_SIZE_PER_CHANNEL = 0x%x\n",NN_MEM_SIZE_PER_CHANNEL);
	printf("  MEMW_BASE = 0x%x\n",MEMW_BASE);
	printf("  MEMI_BASE = 0x%x\n",MEMI_BASE);
	printf("  DMA_MAC_OUT_ARRAY = 0x%x\n\n",base_mac_out_addr);

	this->Reset();
}

TDmaMult::~TDmaMult(){
}

void TDmaMult::Reset(){
	int i;
    // all relevant data go to their initial value at this state
	_dma_state = DmaState::WAIT_CONFIG_STALL;

	// get the pointer to the base memory position where the MACs store their final values 
	float *ptr = (float *)_mem0->GetMap(_base_mac_out_addr);
	for (i=0;i<SIMD_SIZE;i++){
		*ptr = 0;
		ptr++;
	}
}

DmaState TDmaMult::GetDmaState(){return _dma_state;}

SimulationTime TDmaMult::Run(){
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
	int i;
	switch(_dma_state){	
		case DmaState::WAIT_CONFIG_STALL:{
			for (i=0;i<SIMD_SIZE;i++){
				_reg_mac[i] = 0;
			}
			}break;
		case DmaState::COPY_FROM_MEM:{
			if (_mul_ready == 0x1){
				for (i=0;i<SIMD_SIZE;i++){
					_reg_mac[i] += _reg_mul[i];
				}
			} 
			}break;
		case DmaState::COPY_TO_CPU:{
			float *ptr = (float *)_mem0->GetMap(_base_mac_out_addr);
			for (i=0;i<SIMD_SIZE;i++){
				//printf ("MAC[%d]: %f - 0x%p\n", i, _reg_mac[i], &(_reg_mac[i]));
				*ptr = _reg_mac[i];  // send the final result back to the processor
				ptr++;
			}
			}break;
		case DmaState::FLUSH:
			break;
	}
}
#pragma GCC diagnostic pop

// 2rd pipeline stage - do the mult
void TDmaMult::DoMult(){
	int i;
	if (_dma_state == DmaState::WAIT_CONFIG_STALL){
		for (i=0;i<SIMD_SIZE;i++){
			_reg_mul[i] = 0; // restart register 
		}		
		_mul_ready = 0;
	}
	else {
		if (_mul_loaded == 0x1){
			for (i=0;i<SIMD_SIZE;i++){
				_reg_mul[i] = _op1[i] * _op2[i]; // mult
			}				
			_mul_ready = 1;
		}else{
			_mul_ready = 0;
		}
	}
}

// 1st pipeline stage - read the memories and load the mult operands
void TDmaMult::ReadData(){
	int i;
	//send state machine
	switch(_dma_state){
		//wait the cpu to configure the ni
		case DmaState::WAIT_CONFIG_STALL:{
			if(_sig_dma_prog->Read() == 0x1){
				_sig_stall->Write(0x1);        //raise stall
				_mul_loaded = 0;   // raised when the mul can be executed
				// reading data sent from the proc to the DMA
				_burst_size = _sig_burst_size->Read();
				if (_burst_size > NN_MEM_SIZE_PER_CHANNEL){
					stringstream s;
					s << this->GetName() << ": burst size exedded the NN memory capacity.";
					throw std::runtime_error(s.str());
				} 
//				_w_mem_idx = _sig_nn_size->Read();
//				_i_mem_idx = _sig_out_size->Read();
				// init counters used for burst mode operation
				_mem_idx = 0;
				_remaining = _burst_size;
				_dma_state = DmaState::COPY_FROM_MEM; //change states
			}
			
		} break;

		//copy data from the NN memory to the internal MAC registers
		case DmaState::COPY_FROM_MEM:{
			
			if(_remaining > 0){
				int8_t * w_ptr, * i_ptr;

				for (i=0;i<SIMD_SIZE;i++){
					w_ptr = _mem0->GetMap(_memW[i]+_mem_idx);
					_op1[i] = *(float*)w_ptr;
					i_ptr = _mem0->GetMap(_memI[i]+_mem_idx);
					_op2[i]  = *(float*)i_ptr;
					//if ( _op1[i] != 0.0f)
					//	printf ("OPs[%d %d]: %f %f\n", _mem_idx, i, _op1[i], _op2[i]);
				}
				//signal to the next pipiline stage
				_mul_loaded = 1; 
				// updating counters used for burst mode operation
				_remaining--; //one less packet to send
				_mem_idx +=4;
			}else{
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
			_dma_state = DmaState::WAIT_CONFIG_STALL;
			// TODO multiple drivers to signal _sig_dma_prog !!! implement a handshare protocol between proc and dma 
			_sig_dma_prog->Write(0x0);     //lower the start signal . 
			_sig_stall->Write(0x0);        //lowering stall and giving the control back to the processor
			break;
	}
}
