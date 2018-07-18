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
#include <MemoryModel.h>
#include <inttypes.h>

MemoryType MemoryHelper::Create(uint32_t size, bool wipe){

    //TODO: abstract the underlying type    
    MemoryType mem = new int8_t[size];

    if(wipe) MemoryHelper::Wipe(mem, 0, size);

    return mem;
}

void MemoryHelper::Write(MemoryType mem, uint32_t addr, uint8_t* data, uint32_t length){

    //TODO: investigate memcpy performance
	for(int i = 0; i < length; i++){
		mem[addr] = data[i];
		addr++;		
	}
}

void MemoryHelper::Read (MemoryType mem, uint32_t addr, uint8_t* buffer, uint32_t length){
	
	//TODO: investigate memcpy performance
	for(int i = 0; i < length; i++){
		buffer[i] = mem[addr];
		addr++;		
	}
}
		
void MemoryHelper::Wipe(MemoryType mem, uint32_t base, uint32_t size){
	
    //TODO: investigate memcpy, zero fill and other methods
    //for filling the memory with zeroes.
	for(int i = base; i < size; i++)	
		mem[i] = 0x00;
	
}

void MemoryHelper::LoadBin(MemoryType mem, std::string filename, uint32_t base, uint32_t size){

    //TODO: not sure it is the best performatic way
	std::ifstream f(filename, std::ios::binary | std::ios::in | std::ios::out);
	
	if(f.is_open()){
	
		f.read((char*)&mem[base], sizeof(mem[0]) * size);
		f.close();
	}else{
	    //TODO: surround with try-catch instead of printing
		std::cout << "Unable to load '" << filename << "'." << std::endl;
	}
}

void MemoryHelper::Dump(MemoryType mem, uint32_t base, uint32_t length){
	
	printf("--- mem dump:\n");
	
	//fix for printing only 4-digit values
	int mask = 0x000000FF;
	
	for(int i = base; i < length; i+= 16){

		//TODO: fix check on unaligned files
		if(mem[i+1]  + mem[i  ]  + mem[i+3]  + mem[i+2] 
		 + mem[i+5]  + mem[i+4]  + mem[i+7]  + mem[i+6]
 		 + mem[i+9]  + mem[i+8]  + mem[i+11] + mem[i+10]
 		 + mem[i+13] + mem[i+12] + mem[i+15] + mem[i+14] != 0){

	
			printf("%07x ", i);	
			for(int j = 0; j < 16; j+= 2) printf("%02x%02x ", mem[i+j+1]  & mask, mem[i+j] & mask);
			printf("\n");
		}
	}
	
	printf("--- eod:\n");
}

