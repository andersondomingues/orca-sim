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

//simulation API
#include <Event.h>
#include <Simulator.h>

//mem mapping and config.
#include <Gba.h>

//model API
#include <UMemory.h>
#include <TArm7TDMI.h>

int main(int argc, char** argv){

	//memory modules
	UMemory* mem_sysrom = new UMemory(RAM_SYSROM_BASE, RAM_SYSROM_SIZE);
	UMemory* mem_sysrom = new UMemory(RAM_EW_BASE, RAM_EW_SIZE);
	UMemory* mem_sysrom = new UMemory(RAM_IW_BASE, RAM_IW_SIZE);
	UMemory* mem_sysrom = new UMemory(RAM_IO_BASE, RAM_IO_SIZE);
	UMemory* mem_sysrom = new UMemory(RAM_PAL_BASE, RAM_PAL_SIZE);	
	UMemory* mem_sysrom = new UMemory(RAM_VRAM_BASE, RAM_VRAM_SIZE);
	UMemory* mem_sysrom = new UMemory(RAM_OAM_BASE, RAM_OAM_SIZE);	
	UMemory* mem_sysrom = new UMemory(RAM_PAK_BASE, RAM_PAK_SIZE);	
	UMemory* mem_sysrom = new UMemory(RAM_CART_BASE, RAM_CART_SIZE);

	TArm7TDMI* arm7 = new TArm7TDMI("test-arm-elf");

	//instantiate simulation
	Simulator* s = new Simulator();
	s->Schedule(Event(1, arm7cpu));
	
	//keep simulating until something happen
	try{
		s->Run(1);
	}catch(std::runtime_error& e){
		std::cout << e.what() << std::endl;
	}
	
	std::cout << std::flush;
}
