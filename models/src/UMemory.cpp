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
#include <string>
#include <inttypes.h>

#include <UMemory.h>

UMemory::UMemory(std::string name, uint32_t size, uint32_t sram_base, bool wipe, std::string binname) : UntimedModel(name){
	
	_mem = new MemoryType[size];
	_length = size;
	_sram_base = sram_base;

    if(wipe) 
	   	this->Wipe(0, size);

	if(binname != "")
		this->LoadBin(binname, 0, size);
}

void UMemory::Write(uint32_t addr, MemoryType* data, uint32_t length){

    //same performance as memcpy but library independent
	for(uint32_t i = 0; i < length; i++){
		_mem[addr] = data[i];
		addr++;		
	}
}

void UMemory::Read(uint32_t addr, MemoryType* buffer, uint32_t length){
	
	//same performance as memcpy but library independent
	for(uint32_t i = 0; i < length; i++){
		buffer[i] = _mem[addr];
		addr++;
	}
}

		
void UMemory::Wipe(uint32_t base, uint32_t size){
	
    //TODO: investigate memcpy, zero fill and other methods
    //for filling the UMemory with zeroes.
	for(uint32_t i = base; i < size; i++)	
		_mem[i] = 0x00;
	
}

void UMemory::LoadBin(std::string filename, uint32_t base, uint32_t size){

    //TODO: not sure it is the best performatic way
	std::ifstream f(filename, std::ios::binary | std::ios::in | std::ios::out);
	
	if(f.is_open()){
		f.read((char*)&_mem[base], sizeof(_mem[0]) * size);
		f.close();
	}else{
	    //TODO: surround with try-catch instead of printing
		std::string err_msg = this->GetName() + ": unable to load '" + filename + "'.";
		throw std::runtime_error(err_msg);		
	}
}

void UMemory::Dump(uint32_t base, uint32_t length){
	
	printf("--- mem dump:\n");
	
	//fix for printing only 4-digit values
	int mask = 0x000000FF;
	
	for(uint32_t i = base; i < length; i+= 16){

		//TODO: fix check on unaligned files
		if(_mem[i+1]  + _mem[i  ]  + _mem[i+3]  + _mem[i+2] 
		 + _mem[i+5]  + _mem[i+4]  + _mem[i+7]  + _mem[i+6]
 		 + _mem[i+9]  + _mem[i+8]  + _mem[i+11] + _mem[i+10]
 		 + _mem[i+13] + _mem[i+12] + _mem[i+15] + _mem[i+14] != 0){

	
			printf("%07x ", i);	
			for(int j = 0; j < 16; j+= 2) printf("%02x%02x ", _mem[i+j+1]  & mask, _mem[i+j] & mask);
			printf("\n");
		}
	}
	
	printf("--- eod:\n");
}

//TODO:remove it as soon as possible
MemoryType* UMemory::GetMemPtr(){
	return _mem;
}

UMemory::~UMemory(){
	delete(_mem);
}

void UMemory::Reset(){
	delete(_mem);
	_mem = new MemoryType[_length];
}