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
    _ob  = new Buffer(name + ": ob");
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
Buffer* DmniModel::GetOutputBuffer(){ return _ob; }
Buffer* DmniModel::GetInputBuffer(){ return _ib; }

//setters
void DmniModel::SetAddress(uint32_t* add){ _mma_addr = add; }
void DmniModel::SetLength(uint32_t* add){ _mma_len  = add; }
void DmniModel::SetOperation(uint32_t* v){ _mma_op = v; }
void DmniModel::SetIntr(bool* b){ _intr = b;}

void DmniModel::SetMemoryModel(MemoryModel* m){ _mem = m; }
void DmniModel::SetInputBuffer(Buffer* b){ _ib = b; }

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
        
            if(_send_active == true || ((_timer >= DMNI_TIMER) && send_active == true)){
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

void DmniModel::CycleSend(){

    switch(_send_state){
        
        case SendState::WAIT:{
            if(_start == true && *_mma_op == OP_SEND){
                _send_addr = *_mma_addr;
                _send_len  = *_mma_len;
                *_mma_op = OP_NONE;
            }
            break;
        }
        case SendState::COPY_FROM_MEM:{
            
            uint32_t data;
            _mem->Read(_send_addr, (int8_t*)&data, 4);
            _ib->push(data);
            _send_addr = (_send_addr + 4);
            _send_len  = (_send_len -1);
            if(_send_len == 0)
                _send_state = SendState::FINISH;
            break;
        }
        case SendState::FINISH:{
            _send_addr = 0;
            _send_len = 0;
            _send_state = SendState::WAIT;
            break;
        }
    }
}


/**
 * @brief Mimics the receive process of the dmni module */
void DmniModel::CycleReceive(){

    switch(_recv_state){
        case RecvState::WAIT:{
            if(_start == true && *_mma_op == OP_RECV){
                _recv_addr = *_mma_addr;
                _recv_len  = *_mma_len;
                _recv_state = RecvState::COPY_TO_MEM;
            }
            break;
        }
        
        case RecvState::COPY_TO_MEM:{
            if(_read_enable == true){
                uint32_t data = _ib->top();
                _mem->Write(_recv_addr, (int8_t*)&data, 4);
                _ib->pop();
                _recv_len--;
                
                if(_recv_len == 0) _recv_state = RecvState::FINISH;
            }
            break;
        }
        
        case RecvState::FINISH:
            _recv_state = RecvState::WAIT;
        
        break;
        
        
    }        
}

/**
 * @brief Free allocated memory if any
 */
DmniModel::~DmniModel(){}
