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
    this->Reset();
}


void DmniModel::Reset(){

    //arbiter reset
    _read_enable  = false;
    _write_enable = false;
    _prio = false;
    _timer = 0;

    _arbState = ArbiterState::ROUND;

    //receive reset
    _first = 0;
    _last  = 0;
    _payload_size = 0;
    _nocState = NocState::HEADER;
    _add_buffer    = false;
    _receive_active_2 = false;
    
    _recvState = RecvState::WAIT;
    _recv_address = 0;
    _recv_size    = 0;
    
    for(uint32_t i = 0; i < TAM_BUFFER_DMNI; i++)
        _is_header[i] = false;
    _mem_byte_we  = 0;
    _intr_counter_temp = 0;
    
    //reset sender 
    _sendState = SendState::WAIT;
    _send_active_2 = false;
    _tx = false;
    _send_size = 0;
    _send_size_2 = 0;
    _send_address = 0;
    _send_address_2 = 0;
    _data_out = 0  ;
}

/**
 * @brief Implementation of the Run method from
 * the Proccess abstract class.
 * @return The next time to schedule the event.*/
unsigned long long DmniModel::Run(){

	//TODO:parallel for?
	this->CycleArbiter();
	this->CycleReceive();
	this->CycleSend();
	this->CycleConfigure();
	
	//TODO: after cycling commit
	//reason: state changes at the end of cycle, so we
	//cannot allow, for example, the arbiter to commit
	//changes to shared signals until the end cycle, 
	//because it would make receiver, send and config
	//to assume future values instead of current values 
	//for these shared signals.
    
    //combinational hardware
    _mem_address = (_write_enable) ? _send_address : _recv_address;
    _credit_o = _slot_available;
    _slot_available = !(_first == _last && _add_buffer);
    _read_av        = !(_first == _last && !_add_buffer);
    _send_active    = _send_active_2;
    _receive_active = _receive_active_2;
    _intr_count  = _intr_counter_temp;
    _intr = (_intr_counter_temp > 0);
    
    //for now, the tick is the same for all 
    //hardware.
    return 1;
}


/**
 * @brief Mimic the behavior of the receiver module
 * from the real DMNI
 */
void DmniModel::CycleReceive(){
	
    if(_rx && _slot_available){
        _bufferr[_last] = *_data_in;
        _add_buffer = true;
        _last++;
    
        //read from noc
        switch(_nocState){
            
            case NocState::HEADER:
                _intr_counter_temp++;
                _is_header[_last] = true;   
                _nocState = NocState::PAYLOAD;
                break;
            
            case NocState::PAYLOAD:
                _is_header[_last] = false;
                _payload_size = (*_data_in) -1;
                _nocState = NocState::DATA;
                break;
                
            case NocState::DATA:
                _is_header[_last] = false;
                if(_payload_size == 0) _nocState = NocState::HEADER;
                else _payload_size++;
                break;
        }
    }
    
    //write to memory
    switch(_recvState){
        
        case RecvState::WAIT:
            if(_start && _operation){
                _recv_address = _address - WORD_SIZE;
                _recv_size = _size -1;
                if(_is_header[_first] && _intr_counter_temp > 0)
                    _intr_counter_temp--;
                _receive_active_2 = true;
                _recvState = RecvState::COPY_TO_MEM;
            }
            break;
            
        case RecvState::COPY_TO_MEM:
            if(_read_enable && _read_av){
                _mem_byte_we = 15; // "1111", all bits on
                _mem_data_write = _bufferr[_first];
                _first++;
                _add_buffer = false;
                _recv_address = _recv_address + WORD_SIZE;
                _recv_size--;
                if(_recv_size == 0)
                    _recvState = RecvState::FINISH;
                
            }else{
                _mem_byte_we = 0; // "0000", all bits off
            }
            break;
            
        case RecvState::FINISH:
            _receive_active_2 = false;
            _mem_byte_we = 0; //"0000", all bytes off 
            _recv_address = 0;
            _recv_size = 0;
            _recvState = RecvState::WAIT;
            break;
    }
}

/**
 * @brief Mimic the behavior of the arbiter module 
 * from the real DMNI. */
void DmniModel::CycleArbiter(){

	//arbiter state machine
	switch(_arbState){
	
		//ROUND STATE (means "whatever comes first")
		case ArbiterState::ROUND:
		
			if(_prio == false){
				if(_recvState == RecvState::COPY_TO_MEM){
					_arbState = ArbiterState::RECEIVE;
					_read_enable = true;
				}else if(_send_active_2){
					_arbState = ArbiterState::SEND;
					_write_enable = true;
				}
			}else{
				if(_send_active_2){
					_arbState = ArbiterState::SEND;
					_write_enable = true;
				}else if(_recvState == RecvState::COPY_TO_MEM){
					_arbState = ArbiterState::RECEIVE;
					_read_enable = true;				
				}
			}	
			break;
	
		//SEND STATE
		case ArbiterState::SEND:
			if(_sendState == SendState::FINISH || (_timer == DMNI_TIMER && _receive_active_2 == true)){
				_timer = 0;
				_arbState = ArbiterState::ROUND;
				_read_enable = false;
				_prio = !_prio;	
			}else{
				_timer++;
			}		
			break;
	
		//RECV STATE
		case ArbiterState::RECEIVE:
			if(_recvState == RecvState::FINISH || (_timer == DMNI_TIMER && _send_active_2 == true)){
				_timer = 0;
				_arbState = ArbiterState::ROUND;
				_read_enable = false;
				_prio = !_prio;
			}else{
				_timer++;
			}
            break;
	}

}

/**
 * @brief mimic the configuration proccess 
 * of the real DMNI module */
void DmniModel::CycleConfigure(){
    
    if (_set_address) {
        _address = *_config_data;
        _address_2 = 0;
    }else if (_set_address_2){
        _address_2 = *_config_data;
    }else if (_set_size){
        _size = *_config_data;
        _size_2 = 0;
    }else if (_set_size_2){
        _size_2 = *_config_data;
    }else if (_set_op){
        
        //it is implemented as config_data(0), but 
        //we keep all bits off here, for convention
        //TODO: rework for using mask (config_data & 0xA000?)
        _operation = (*_config_data == 0);
    }
}

/**
 * @brief mimic the behaviour of Sender module from the 
 * real DMNI module. */
void DmniModel::CycleSend(){
    
    switch(_sendState){
        
        case SendState::WAIT:
            if(_start && _operation){
                _send_address = _address;
                _send_address_2 = _address_2;
                _send_size = _size;
                _send_size_2 = _size_2;
                _send_active_2 = true;
                _sendState = SendState::LOAD;
            }
            break;
            
        case SendState::LOAD:
            if(_credit_i && _write_enable){
                _send_address = _send_address + WORD_SIZE;
                _sendState = SendState::COPY_FROM_MEM;
            }
            break;
        
        case SendState::FINISH:
            _send_active_2  = false;
            _send_address   = 0;
            _send_address_2 = 0;
            _send_size   = 0;
            _send_size_2 = 0;
            _sendState = SendState::WAIT;
        
        case SendState::COPY_FROM_MEM:
            if(_credit_i && _write_enable){
                if(_send_size > 0){
                    _tx = true;
                    _data_out = *_mem_data_read;
                    _send_address = _send_address + WORD_SIZE;
                    _send_size--;
                }else if (_send_size_2 > 0){
                    _send_size = _send_size_2;
                    _send_size_2 = 0;
                    _tx = false;
                    
                    //send_address_2(30 downto 28) = "000"
                    _send_address = ((_send_address_2 >> 28) == 16 || (_send_address_2 >> 28) == 0)
                        ? _send_address_2
                        : _send_address_2 - WORD_SIZE;
                        
                    _sendState = SendState::LOAD;
                } else {
                    _tx = false;
                    _sendState = SendState::FINISH;
                }  
            }else{
                if (!_credit_i){
                  _send_size++;
                  _send_address = _send_address - (2 * WORD_SIZE);
                }else{
                  _send_address = _send_address - WORD_SIZE;
                }
                _tx = false;
                _sendState = SendState::LOAD;
            }
            break;
    }
    
}

/**
 * @brief Free allocated memory if any
 */
DmniModel::~DmniModel(){}

/**
 * @brief PortMap function mimics the port mapping
 * from VHDL. TODO:implement into abstract class
 * to enforce its implementation
 * @param set_address
 * @param set_address_2
 * @param set_size
 * @param set_size_2
 * @param set_op
 * @param start
 * @param config_data
 * @param mem_data_read
 * @param credit_i
 * @param rx
 * @param data_in */
void DmniModel::PortMap(
    //configuration if
    bool* set_address,
    bool* set_address_2,
    bool* set_size,
    bool* set_size_2,
    bool* set_op,
    bool* start,
    uint32_t* config_data,
    
    //memory if
    uint32_t* mem_data_read,
    
    //noc if (local port)
    bool* credit_i,
    bool* rx,
    RegFlit* data_in
){
    _set_address   = set_address;
    _set_address_2 = set_address_2;
    _set_size   = set_size;
    _set_size_2 = set_size_2;
    _set_op = set_op;
    _start = start;
    _config_data = config_data;
    
    _mem_data_read = mem_data_read;
    
    _credit_i = credit_i;
    _rx = rx;
    _data_in = data_in;
}

//getters
bool* DmniModel::GetIntr(){ return &_intr; }
bool* DmniModel::GetSendActive(){ return &_send_active;}
bool* DmniModel::GetReceiveActive(){ return &_receive_active;}

//memory
uint32_t* DmniModel::GetMemAddress(){ return &_mem_address;}
uint32_t* DmniModel::GetMemDataWrite(){ return &_mem_data_write;}
uint8_t*  DmniModel::GetMemByteWe(){ return &_mem_byte_we;}

//noc interface
bool* DmniModel::GetTx(){ return &_tx;}
RegFlit* DmniModel::GetDataOut(){ return &_data_out;}
bool* DmniModel::GetCreditO(){ return &_credit_o;}
