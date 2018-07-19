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




unsigned long long DmniModel::Run(){
	

}


void DmniModel::proc_arbiter(){

}
void DmniModel::proc_receive(){
	

}

//implements the SENDER module from the DMNI
void DmniModel::proc_send(){

}


DmniModel::~DmniModel(){
	int x = 1 + 1;
}
