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
#include <USignal.h>

/**
 * @brief Instantiate a new memory module.
 * @param name A name that uniquely identify the instantiated module (can be empty)
 * @param size Total amount of bytes allocated to the memory
 * @param sram_base The base address for the memory. The base addres is the first address
 * @param wipe Clean the memory after creation (zero fill)
 * @param binname A binary file to load into memory after instantiation (optional)
 */
UMemory::UMemory(std::string name, uint32_t size, uint32_t sram_base, bool wipe, std::string binname) : UntimedModel(name){
	
	_mem = new MemoryType[size];

	_length = size;
	_sram_base = sram_base;

    if(wipe) 
	   	this->Wipe();

	if(binname != "")
		this->LoadBin(binname, _sram_base, _length);

}

#ifdef MEMORY_ENABLE_COUNTERS
void UMemory::InitCounters(uint32_t store_counter_addr, uint32_t load_counter_addr){
	_counter_nstore = new USignal<uint32_t>(store_counter_addr, GetName() + ".counters.store");
	_counter_nload = new USignal<uint32_t>(load_counter_addr, GetName() + ".counters.load");
}

USignal<uint32_t>* UMemory::GetSignalCounterStore(){
	return _counter_nstore;
}

USignal<uint32_t>* UMemory::GetSignalCounterLoad(){
	return _counter_nload;
}
#endif


/**
 * @brief Writes a bytestream to the memory
 * @param addr Starting addres of range to be written
 * @param data Address to copy data from
 * @param length The number of bytes to write
 */
void UMemory::Write(uint32_t addr, MemoryType* data, uint32_t length){

	#ifdef MEMORY_WRITE_ADDRESS_CHECKING
	if(addr < _sram_base){
		stringstream s;
		s << this->GetName() << ": unable to write to addr (0x" << std::hex
			<< addr << ") lower than sram base.";
	
		throw std::runtime_error(s.str());
	}

	if((addr + length -1) > GetLastAddr()){
		stringstream s;
		s << this->GetName() << ": unable to write to addr (0x" << std::hex
			<< addr << ") higher than last mapped address of (0x" << std::hex 
			<< GetLastAddr() << ").";
		throw std::runtime_error(s.str());
	}
	#endif

	#ifdef MEMORY_ENABLE_COUNTERS
	//increment number of stores into nstore counter
	_counter_nstore->Inc(1);
	#endif 

    //same performance as memcpy but library independent
	for(uint32_t i = 0; i < length; i++){
		_mem[addr - _sram_base] = data[i];
		addr++;
	}
}

/**
 * @brief Reads a bytestream from the memory
 * @param addr Starting address to be read
 * @param buffer Place to write the read data
 * @param length Number of bytes to read
 */
void UMemory::Read(uint32_t addr, MemoryType* buffer, uint32_t length){

	#ifdef MEMORY_READ_ADDRESS_CHECKING
	if(addr < _sram_base){
		stringstream s;
		s << this->GetName() << ": unable to read from addr (0x" << std::hex
			<< addr << ") lower than sram base.";
		abort();
		throw std::runtime_error(s.str());
	}

	if((addr + length) - 1 > GetLastAddr()){
		stringstream s;
		s << this->GetName() << ": unable to read from addr (0x" << std::hex
			<< addr << ") higher than last mapped address of (0x" << std::hex 
			<< GetLastAddr() << ").";
		throw std::runtime_error(s.str());
	}
	#endif
	
	//same performance as memcpy but library independent
	for(uint32_t i = 0; i < length; i++)
		buffer[i] = _mem[(addr - _sram_base) + i];
	
	#ifdef MEMORY_ENABLE_COUNTERS
	//increment number of loades into nstore counter
	_counter_nload->Inc(1);
	#endif 
}

/**
 * @brief Fill all memory positions with zeroes.
 */
void UMemory::Wipe(){
	this->Wipe(_sram_base, _length);
}

/**
 * @brief 
 * @param addr
 * @return 
 */
MemoryType* UMemory::GetMap(uint32_t addr){
	
	if(addr < _sram_base){
		stringstream s;
		s << this->GetName() << ": unable to map from to addr (0x" << std::hex
			<< addr << ") lower than sram base.";
		throw std::runtime_error(s.str());
	}

	if(addr > GetLastAddr()){
		stringstream s;
		s << this->GetName() << ": unable to map from to addr (0x" << std::hex
			<< addr << ") higher than last mapped address of (0x" << std::hex 
			<< GetLastAddr() << ").";
		throw std::runtime_error(s.str());
	}
	
	return &(_mem[addr - _sram_base]);
}

/**
 * @brief Return the base address 
 * @return the base address
 */
uint32_t UMemory::GetBase(){
	return _sram_base;
}

/**
 * @brief Return the size of the memory
 * @return the size
 */
uint32_t UMemory::GetSize(){
	return _length;
}

/**
 * @brief Return the last address of the memory
 * @return the last address
 */
uint32_t UMemory::GetLastAddr(){
	return (_sram_base + _length) - 1;
}

/**
 * @brief Fill some part of the memory with zeroes
 * @param base The starting address
 * @param size The number of bytes to write to
 */
void UMemory::Wipe(uint32_t base, uint32_t size){
	
	#ifdef MEMORY_WIPE_ADDRESS_CHECKING
	if(base < _sram_base)
		throw std::runtime_error(this->GetName() + ": unable to wipe from base (" + std::to_string(base) + ") lower than sram base .");

	if(base > _sram_base + _length)
		throw std::runtime_error(this->GetName() + ": unable to wipe from base (" + std::to_string(base) + ") higher than sram base .");

	if(base + size > _sram_base + _length)
		throw std::runtime_error(this->GetName() + ": unable to wipe that much length (" + std::to_string(size) + ") higher than sram base + length.");
	#endif
	
	MemoryType* range_ptr = _mem + (base - _sram_base);
	for(uint32_t i = 0; i < size; i++)
		range_ptr[i] = 0;
}

/**
 * @brief Load a binary file into memory
 * @param filename Name of file containing memory data (program)
 * @param base Base address to start writing to
 * @param size Number of bytes to write into memory
 */
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

void UMemory::SaveBin(std::string filename, uint32_t base, uint32_t size){
	
	//std::ofstream f(filename, std::ios::binary | std::ios::in | std::ios::out);
	std::ofstream f(filename, std::ifstream::binary);
	
	if(f.is_open()){
		f.write((char*)&_mem[base -_sram_base], sizeof(_mem[0]) * size);
		f.close();
	}else{
	    //TODO: surround with try-catch instead of printing
		std::string err_msg = this->GetName() + ": unable to save'" + filename + "'.";
		throw std::runtime_error(err_msg);
	}
}

/**
 * @brief Show the content of memory
 */
void UMemory::Dump(){
	this->Dump(0, _length);
}

/**
 * @brief Show the content of memory given some range
 * @param base The starting address to start printing
 * @param length The number of bytes to print
 */
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
			printf("%02x ", _mem[k + l + (base - _sram_base)] & mask );
			if (l == 7) putchar(' ');
		}
		printf(" |");
		for(l = 0; l < 16; l++){
			ch = _mem[k + l + (base - _sram_base)];
			if ((ch >= 32) && (ch <= 126))
				putchar(ch);
			else
				putchar('.');
		}
		putchar('|');
	}
}

/**
 * @brief Dtor.
 */
UMemory::~UMemory(){
	delete [] _mem;
	
	#ifdef MEMORY_ENABLE_COUNTERS
	delete(_counter_nstore);
	delete(_counter_nload);
	#endif
	
}

void UMemory::Reset(){
	//delete(_mem);
	//_mem = new MemoryType[_length];
}
