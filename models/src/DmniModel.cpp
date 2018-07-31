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
#include <DmniModel.h>
#include <Buffer.h>

#include <cstdlib>

DmniModel::DmniModel(std::string name) : Process(name){
    _ob  = new Buffer<FlitType>(name + ": ob");
}

//status wires (memory mapped)
bool send_active;
bool recv_active;

//getters
uint32_t* DmniModel::GetAddress(){ return _mma_addr; }
uint32_t* DmniModel::GetLength() { return _mma_len;  }
uint32_t* DmniModel::GetOperation(){ return _mma_op; }
bool* DmniModel::GetIntr(){ return _intr; }

MemoryModel* DmniModel::GetMemoryModel(){ return _mem; }
Buffer<FlitType>* DmniModel::GetOutputBuffer(){ return _ob; }
Buffer<FlitType>* DmniModel::GetInputBuffer(){ return _ib; }

//setters
void DmniModel::SetAddress(uint32_t* add){ _mma_addr = add; }
void DmniModel::SetLength(uint32_t* add){ _mma_len  = add; }
void DmniModel::SetOperation(uint32_t* v){ _mma_op = v; }
void DmniModel::SetIntr(bool* b){ _intr = b;}

void DmniModel::SetMemoryModel(MemoryModel* m){ _mem = m; }
void DmniModel::SetInputBuffer(Buffer<FlitType>* b){ _ib = b; }

/**
 * @brief reset state
 */
void DmniModel::Reset(){

    //dmni -> proc IF
    *_intr = false;
    
    //proc -> dmni IF
    *_mma_addr = 0;
    *_mma_len  = 0;
    
    
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
unsigned long long DmniModel::Run(){
    CycleArbiter();
    CycleReceive();
    CycleSend();
    return 1;
}

void DmniModel::CycleArbiter(){
    
    switch(_arb_state){
        case ArbiterState::ROUND:{
            
            if(_prio == true){ //RECV
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
            
            if(_send_state == SendState::FINISH || ((_timer >= DMNI_TIMER) && _recv_active == true)){
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
        
            if(_send_active == true || ((_timer >= DMNI_TIMER) && _send_active == true)){
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
void DmniModel::CycleSend(){

    switch(_send_state){

        //In this state, the sender module 
        //wait to be configured through the
        //cpu (wires _mma_op, _mma_addr, _mma_len).
        case SendState::WAIT:{
            if(*_mma_op == OP_SEND){
                _send_addr = *_mma_addr;
                _send_len  = *_mma_len;
                *_mma_op = OP_NONE;
                _send_state = SendState::COPY_FROM_MEM;
            }
            break;
        }
        //In this state, data is copied from the 
        //memory and put into the output buffer (which
        //is, in most cases, connected to a router)
        case SendState::COPY_FROM_MEM:{
            
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
            break;
        }
        //Clean wires and go back to the wait state.This
        //state is not necessary but has been kept to 
        //conform with cycle precision.
        case SendState::FINISH:{
            _send_addr = 0;
            _send_len = 0;
            _send_state = SendState::WAIT;
            break;
        }
    }
}

/**
 * @brief Mimics the Receiver process of the dmni module */
void DmniModel::CycleReceive(){

    switch(_recv_state){
        case RecvState::WAIT:{
            
            if(*_mma_op == OP_RECV){
                _recv_addr = *_mma_addr;
                _recv_len  = *_mma_len;
                _recv_state = RecvState::COPY_TO_MEM;
                *_mma_op = OP_NONE;

                std::cout << "WAIT" << std::endl;
                
            //if operation is other than receive from
            //noc, keep raise the interrupt signal
            }else if(_ib->size() > 0){
                *_intr = true;
            }
            
            break;
        }
        
        case RecvState::COPY_TO_MEM:{
            std::cout << "COPY" << std::endl;
            if(_read_enable == true){
                
                //FlitType is a 2-byte type, so we
                //need to write twice.
                FlitType flit = _ib->top();
                _mem->Write(_recv_addr, (int8_t*)&flit, 2);
                _ib->pop();
                _recv_len--;
                
                if(_recv_len == 0) 
                    _recv_state = RecvState::FINISH;
            }
            break;
        }
        
        case RecvState::FINISH:{
            std::cout << "FINISH" << std::endl;
            _recv_state = RecvState::WAIT;
            break;
        }
    }        
}

/**
 * @brief Free allocated memory if any
 */
DmniModel::~DmniModel(){}
