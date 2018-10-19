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
#include <sstream>
#include <inttypes.h>
#include <UMemory.h>

//#define NOGUARDS 1

UMemory::UMemory(std::string name, uint32_t size, uint32_t sram_base, bool wipe, std::string binname) : UntimedModel(name){
	
	_mem = new MemoryType[size];
	_length = size;
	_sram_base = sram_base;

    if(wipe) 
	   	this->Wipe();

	if(binname != "")
		this->LoadBin(binname, _sram_base, _length);
}

void UMemory::Write(uint32_t addr, MemoryType* data, uint32_t length){

	#ifndef NOGUARDS
	if(addr < _sram_base){
		stringstream s;
		s << this->GetName() << ": unable to write to addr (0x" << std::hex
			<< addr << ") lower than sram base.";
	
		throw std::runtime_error(s.str());
	}

	if(addr > _sram_base + _length){
		stringstream s;
		s << this->GetName() << ": unable to write to addr (0x" << std::hex
			<< addr << ") higher than sram base + mem_size.";
		throw std::runtime_error(s.str());
	}
	#endif

	/*
	int xxx = 0;
	if(addr == 0x3fffff80 && *data == '"'){
		xxx = 1;
		std::cout << std::hex << addr << "|" << std::hex << length << "|" << std::hex << *data << std::endl;
	}
		*/
			

    //same performance as memcpy but library independent
	for(uint32_t i = 0; i < length; i++){
		_mem[addr - _sram_base] = data[i];
		addr++;
	}
	/*
						
	if(xxx == 1){
		std::cout << std::hex << (addr -1)<< "|" << std::hex << length << "|" << std::hex << _mem[addr - _sram_base -1] << std::endl;
	}
	*/
}

void UMemory::Read(uint32_t addr, MemoryType* buffer, uint32_t length){

	#ifndef NOGUARDS
	if(addr < _sram_base){
		stringstream s;
		s << this->GetName() << ": unable to read from to addr (0x" << std::hex
			<< addr << ") lower than sram base.";
		throw std::runtime_error(s.str());
	}

	if(addr > _sram_base + _length){
		stringstream s;
		s << this->GetName() << ": unable to read from to addr (0x" << std::hex
			<< addr << ") higher than sram base + mem_size.";
		throw std::runtime_error(s.str());
	}
	#endif
	
	//same performance as memcpy but library independent
	for(uint32_t i = 0; i < length; i++){
		buffer[i] = _mem[addr - _sram_base];
		addr++;
	}
}

//alias
void UMemory::Wipe(){
	this->Wipe(_sram_base, _length);
}


uint32_t UMemory::GetBase(){
	return _sram_base;
}

uint32_t UMemory::GetSize(){
	return _length;
}

void UMemory::Wipe(uint32_t base, uint32_t size){
	
	#ifndef NOGUARDS
	if(base < _sram_base)
		throw std::runtime_error(this->GetName() + ": unable to wipe from base (" + std::to_string(base) + ") lower than sram base .");

	if(base > _sram_base + _length)
		throw std::runtime_error(this->GetName() + ": unable to wipe from base (" + std::to_string(base) + ") higher than sram base + length.");
		
	if(size > _length)
		throw std::runtime_error(this->GetName() + ": unable to wipe beyound memory size (" + std::to_string(size) + ", but memory size is " + std::to_string(_length));
	#endif
	
	for(uint32_t i = base; i < size; i++)	
		_mem[i] = 0x00;
	
}

void UMemory::LoadBin(std::string filename, uint32_t base, uint32_t size){

    //TODO: not sure it is the best performatic way
	std::ifstream f(filename, std::ios::binary | std::ios::in | std::ios::out);
	
	if(f.is_open()){
		f.read((char*)&_mem[base -_sram_base], sizeof(_mem[0]) * size);
		f.close();
	}else{
	    //TODO: surround with try-catch instead of printing
		std::string err_msg = this->GetName() + ": unable to load '" + filename + "'.";
		throw std::runtime_error(err_msg);		
	}
}

void UMemory::Dump(){
	this->Dump(0, _length);
}

void UMemory::Dump(uint32_t base, uint32_t length){
	uint32_t k, l;
	
	//mask is necessary to correct a bug(?) when printing
	//negative hexas.
	uint32_t mask = 0x000000FF; 
	int8_t ch;
	
	//uint32_t* memptr = (uint32_t*)_mem;
	//uint32_t  len = _length / 4;
	for(k = 0; k < length; k += 16){
		printf("\n%08x ", base + k);
		for(l = 0; l < 16; l++){
			printf("%02x ", _mem[k + l] & mask );
			if (l == 7) putchar(' ');
		}
		printf(" |");
		for(l = 0; l < 16; l++){
			ch = _mem[k + l];
			if ((ch >= 32) && (ch <= 126))
				putchar(ch);
			else
				putchar('.');
		}
		putchar('|');
	}
}

UMemory::~UMemory(){
	delete(_mem);
}

void UMemory::Reset(){
	delete(_mem);
	_mem = new MemoryType[_length];
}