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
#include <iostream>

//Modules from UAPI does not require SAPI to run
#include <UMemory.h>

#define CYCLES_TO_SIM 10000000
#define MEM_SIZE 0x80
#define MEM_BASE 0x90000000

int main(int argc, char** argv){

	std::cout << "MEMORY TEST" << std::endl;
	std::cout << "Simulation step set to " << CYCLES_TO_SIM << " cycles." << std::endl;

	//instantiate a new memory module
	UMemory* mem0 = new UMemory("X.mem0", MEM_SIZE, MEM_BASE);
	
	//TEST 1: Try to write below memory range
	bool test1_failed = true;
	try{
		MemoryType t = (mem0->GetBase() - 1);
		mem0->Write(t, &t, 1);
	}catch(...){
		test1_failed = false;
	}
	
	std::cout << "Test1: " << ((test1_failed) ? "failed" : "passed") << std::endl;

	//TEST 2: Try to write above memory range
	bool test2_failed = true;
	try{
		MemoryType t = (mem0->GetBase() + mem0->GetSize() + 1);
		mem0->Write(t, &t, 1);
	}catch(...){
		test2_failed = false;
	}
	
	std::cout << "Test2: " << ((test2_failed) ? "failed" : "passed") << std::endl;
	
	//TEST 3: leakage test
	for(unsigned int i = 0; i < mem0->GetSize(); i++){
		MemoryType t = i + 1;
		mem0->Write(mem0->GetBase() + i, &t, 1);
	}
	
	mem0->Dump();
	std::cout << std::endl;
	
	//TEST 4: rewrite
	for(unsigned int i = 0; i < mem0->GetSize(); i++){
		MemoryType t = mem0->GetSize() - i;
		mem0->Write(mem0->GetBase() + i, &t, 1);
	}
	
	mem0->Dump();
	
	std::cout << std::flush;
}