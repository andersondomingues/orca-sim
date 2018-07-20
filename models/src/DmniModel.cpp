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

DmniModel::DmniModel(std::string name, MemoryModel* m, MemoryAddr intr, MemoryAddr mmr) : Process(name){
	
	this->mem = m;     //memory model ptr
	this->intr = intr; //interrupt addr
	this->mmr = mmr;   //mmr addr
	
	//clean buffers
	this->ib = nullptr;
	this->ob = new Buffer();
}

//getters and setters for buffers
void DmniModel::SetOutputBuffer(Buffer* b){
	this->ob = b;
}

void DmniModel::SetInputBuffer(Buffer* b){
	this->ib = b;
}

Buffer* DmniModel::GetOutputBuffer(Buffer* b){
	return this->ob;
}

Buffer* DmniModel::GetInputBuffer(Buffer* b){
	return this->ib;
}


void DmniModel::reset(){

	//arbiter reset
	this->read_enable  = false;
	this->write_enable = false;
	this->prio = false;
	this->timer = 0;
	
	this->arbState = ArbiterState::ROUND;
	
	//

}

void DmniModel::proc_receive(){
	

}

unsigned long long DmniModel::Run(){

	//TODO:parallel for?
	proc_arbiter();
	proc_receive();
	proc_send();
	proc_config();
	
	//TODO: after cycling commit
	//reason: state changes at the end of cycle, so we
	//cannot allow, for example, the arbiter to commit
	//changes to shared signals until the end cycle, 
	//because it would make receiver, send and config
	//to assume future values instead of current values 
	//for these shared signals.
}


void DmniModel::proc_arbiter(){

	//arbiter state machine
	switch(this->arbState){
	
		//ROUND STATE (means "whatever comes first")
		case ArbiterState::ROUND:
		
			if(this->prio == false){
				if(this->recvState == SendState::COPY_TO_MEM){
					this->arbState = ArbiterState::RECEIVE;
					this->read_enable = true;
				}else if(send_active_2 = true){
					this->arbState = ArbiterState::SEND;
					this->write_enable = true;
				}
			}else{
				if(send_active_2 = true){
					this->arbState = ArbiterState::SEND;
					this->write_enable = true;
				}else if(this->recvState == SendState::COPY_TO_MEM){
					this->arbState = ArbiterState::RECEIVE;
					this->read_enable = true;				
				}
			}	
			break;
	
		//SEND STATE
		case ArbiterState::SEND:
			if(sendState == SendState::FINISH || (this->timer == DMNI_TIMER && receive_active_2 == true)){
				this->timer = 0;
				this->arbState = ArbiterState::ROUND;
				this->read_enable = false;
				this->prio = !this->prio;	
			}else{
				this->timer++;
			}		
			break;
	
		//RECV STATE
		case ArbiterState::RECEIVE:
			if(recvState == SendState::FINISH || (this->timer == DMNI_TIMER && send_active_2 == true)){
				this->timer = 0;
				this->arbState = ArbiterState::ROUND;
				this->read_enable = false;
				this->prio = !this->prio;
			}else{
				this->timer++;
			}
	
		break;
	
	}

}

void DmniModel::proc_config(){

}

//implements the SENDER module from the DMNI
void DmniModel::proc_send(){

}


DmniModel::~DmniModel(){
	int x = 1 + 1;
}
