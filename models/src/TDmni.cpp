/** 
 * This file is part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
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
#include <TDmni.h>

#include <cstdlib>

TDmni::TDmni(std::string name) : TimedModel(name){
    _ob  = new UBuffer<FlitType>(name + ": ob");
	_mem = nullptr;
}

//status wires (memory mapped)
bool send_active;
bool recv_active;

UMemory* TDmni::GetMemoryModel(){ return _mem; }
UBuffer<FlitType>* TDmni::GetOutputBuffer(){ return _ob; }
UBuffer<FlitType>* TDmni::GetInputBuffer(){ return _ib; }

//setters
void TDmni::SetIntr(bool* b){ _intr = b;}
void TDmni::SetMemoryModel(UMemory* m){ _mem = m; }
void TDmni::SetInputBuffer(UBuffer<FlitType>* b){ _ib = b; }

/**
 * @brief reset state
 */
void TDmni::Reset(){
   
    //proc -> dmni IF
    _mma_addr = 0;
    _mma_len  = 0;
	_mma_op   = OP_NONE;
    
	//memory mapped statuses
	if(_mem != nullptr){
		int8_t data = 0;
		//_mem->Write(DMNI_SEND_ACTIVE, &data, 1);
		//_mem->Write(DMNI_RECV_ACTIVE, &data, 1);
		//_mem->Read(DMNI_SIZE,    &_mma_len, 1);  //read 1 word
	}
	
    //arbiter reset
    _read_enable  = false;
    _write_enable = false;
    _timer = 0;
    _prio = false;
    
    _arb_state = ArbiterState::ROUND;
    _send_state = SendState::WAIT;
    _recv_state = RecvState::WAIT;
}

/**
 * @brief Implementation of the Run method from
 * the Proccess abstract class.
 * @return The next time to schedule the event.*/
unsigned long long TDmni::Run(){
    
	//update mma status (currently mapped to memory)
	//_mem->Dump();
	
	//if(this->GetName() == "DMNI_0_2"){
	//_mem->Read(DMNI_ADDRESS, &_mma_addr, 1); //read 1 word
	//	std::cout << std::to_string(_mma_addr) << endl;
	//}
	
	//_mem->Read(DMNI_SIZE,    &_mma_len, 1);  //read 1 word
	//_mem->Read(DMNI_OP,      &_mma_op, 1);   //read 1 word
	
	CycleArbiter();
    CycleReceive();
    CycleSend();
	
	//commit status to memory
	
    return 1;
}

void TDmni::CycleArbiter(){

    switch(_arb_state){
        case ArbiterState::ROUND:{

            if(_prio){ //RECV
                if(_recv_state == RecvState::COPY_TO_MEM){
                    _arb_state = ArbiterState::RECV;
                    _read_enable = true;
                }else if(_send_state == SendState::COPY_FROM_MEM){
                    _arb_state = ArbiterState::SEND;
                    _write_enable = true;
                }
            }else{
                if(_send_state == SendState::COPY_FROM_MEM){
                    _arb_state = ArbiterState::SEND;
                    _write_enable = true;
                }else if(_recv_state == RecvState::COPY_TO_MEM){
                    _arb_state = ArbiterState::RECV;
                    _read_enable = true;
                }
            }
            break;
        }
        case ArbiterState::SEND:{
            
            if(_send_state == SendState::FINISH || ((_timer >= DMNI_TIMER) && (_recv_state == RecvState::COPY_TO_MEM))){
                _timer = 0;
                _arb_state = ArbiterState::ROUND;
                _write_enable = false;
                _prio = !_prio;
            }else{
                _timer++;
            }
            break;
        }
        case ArbiterState::RECV:{

            if(_recv_state == RecvState::FINISH || ((_timer >= DMNI_TIMER) && (_send_state == SendState::COPY_FROM_MEM))){
                _timer = 0;
                _arb_state = ArbiterState::ROUND;
                _read_enable = false;
                _prio = !_prio;
            }else{
                _timer++;
            }
            break;
        }
    }
    
}

/** 
 * @brief Mimic the Sender process of the DMNI. */
void TDmni::CycleSend(){

    switch(_send_state){

        //In this state, the sender module 
        //wait to be configured through the
        //cpu (wires _mma_op, _mma_addr, _mma_len).
        case SendState::WAIT:{
            if(_mma_op == OP_SEND){
                _send_addr = _mma_addr;
                _send_len  = _mma_len;
                _send_state = SendState::COPY_FROM_MEM;
                _write_enable = true;
            }
            break;
        }
        //In this state, data is copied from the 
        //memory and put into the output buffer (which
        //is, in most cases, connected to a router)
        case SendState::COPY_FROM_MEM:{
            
			_mma_op = OP_NONE;
            if(_arb_state == ArbiterState::SEND){
				
                //DMNI operates over FlitType (curretly uint16_t),
                //but memory operates over MemoryType (curently uint8_t).
                //So, we must read 2 bytes at each time. The number of 
                //bytes for the payload must be correctly set from software
                //1st flit) destination and packet length (number of flits)
                //2nd flit) source, service, payload size
                //3rd flit) data ...
                //[x][y][length] [x][y][s][p] [------][------] [------]...
                
                
                //[][][][][][][][] 32
                //[][][][] 16 << flit
                //[][] 8 << mem word
                FlitType flit;
                
                _mem->Read(_send_addr, (int8_t*)(&flit), 2);
                _ob->push(flit);

                _send_addr += 2; //skip 2 positions 
                _send_len--;     //1 less slot to be sent
                
                if(_send_len == 0)
                    _send_state = SendState::FINISH;
            }
            break;
        }
        //Clean wires and go back to the wait state.This
        //state is not necessary but has been kept to 
        //conform with cycle precision.
        case SendState::FINISH:{
            _send_addr = 0;
            _send_len = 0;
            _send_state = SendState::WAIT;
            _write_enable = false;
            break;
        }
    }
}

/**
 * @brief Mimics the Receiver process of the dmni module */
void TDmni::CycleReceive(){

    switch(_recv_state){
		
		//if configured to copy to memory, the 
		//dmni change states and wait for next cycle,
		//otherwise stays stuck in wait state.
        case RecvState::WAIT:{
			
            if(_mma_op == OP_RECV){
                _recv_addr = _mma_addr;
                _recv_len  = _mma_len;                
                _recv_state = RecvState::COPY_TO_MEM;
            }
            break;
        }
        
		//copies to memory only when the arbiter state is RECV
        case RecvState::COPY_TO_MEM:{
            _mma_op = OP_NONE;
			
            if(_arb_state == ArbiterState::RECV){
                
                //2-byte type, so we need to write twice.
                FlitType flit = _ib->top();
							
				_mem->Write(_recv_addr, (int8_t*)&flit, 2);
				
                _ib->pop();
				
				_recv_addr += 2;
                _recv_len--;
                
                if(_recv_len == 0) _recv_state = RecvState::FINISH;
            }
            break;
        }
        
        case RecvState::FINISH:{
            _recv_state = RecvState::WAIT;
            break;
        }
    }        
}


/**
 * @brief Sends data from the memory to the 
 * network router. Data is identified by a
 * starting address and size.
 * @param addr Address in which data begins.
 * @param size Total length o data (size of FlitType) */
void TDmni::CopyFrom(uint32_t addr, uint32_t size){
	_mma_addr = addr;
	_mma_len  = size;
	_mma_op = OP_SEND;
}

/**
 * @brief Receives data from the network router and 
 * stores it into the memory. 
 * @param addr Address to start the writing.
 * @param size Size of data to be written.*/
void TDmni::CopyTo(uint32_t addr, uint32_t size){
	_mma_addr = addr;
	_mma_len  = size;
	_mma_op   = OP_RECV;
}

/**
 * @brief Free allocated memory if any
 */
TDmni::~TDmni(){}
