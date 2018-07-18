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
#include <cstdlib>

DmniModel::DmniModel(string name) : Process(name){
	
	//reset arbiter
	write_enable = 0;
	read_enable  = 0;
	timer = 0;
	prio = 0;
	arbState = ArbiterState::ROUND;
	
	//reset send
	send_active_2 <= 0;
	tx = 0;
	send_size      = 0;
	send_size_2    = 0;
	send_address   = 0;
	send_address_2 = 0;
	sendState = SendState::WAIT;

	//reset recv
	first        = 0;
	last         = 0;
	payload_size = 0;
    nocState     = NocState::HEADER;
    
    add_buffer       = false;
    receive_active_2 = false;
    recvState = SendState::WAIT;
    
    recv_address = 0;
    recv_size    = 0;
    //mem_data_write <= (others=> '0');
    //is_header <= (others=> '0');
    intr_counter_temp = 0;
	//mem_byte_we <= (others=> '0'); 	
}

unsigned long long DmniModel::Run(){
	//TODO: parallel for 
	this->proc_arbiter();
	this->proc_send();
	this->proc_receive();
	return 10;
}

void DmniModel::proc_arbiter(){

	switch(this->arbState){
		case ArbiterState::ROUND:
			break;
		case ArbiterState::SEND:
			break;
		case ArbiterState::RECEIVE:
			break;	
	}

}
void DmniModel::proc_receive(){
	
	//recv from noc (1 cycle)
	//if RX = 1, SLOT_AVAILABLE = 1
	
	switch(this->nocState){

		case NocState::HEADER:
			this->intr_counter_temp++;
			this->is_header[last] = true;
			this->nocState = NocState::PAYLOAD;
			break;
			
		case NocState::PAYLOAD:
			this->is_header[last] = false;
			this->payload_size = data_in - 1;
			this->nocState = NocState::DATA;
			break;
			
		case NocState::DATA:
			this->is_header[last] = false;

			if(payload_size == 0) 
				nocState = NocState::HEADER;
			else
				payload_size--;
			break;	
	
	}
	
	//write to memory (1 more cycle)
	switch(this->recvState){
		
	
	}


}

//implements the SENDER module from the DMNI
void DmniModel::proc_send(){

}


DmniModel::~DmniModel(){
	int x = 1 + 1;
}
