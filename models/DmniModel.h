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
#ifndef __DMNI_H
#define __DMNI_H

//std API
#include <iostream>

//simulator API
#include <Process.h>
#include <Buffer.h>
//model API
#include <MemoryModel.h>

#define TAM_BUFFER_DMNI 
#define DMNI_TIMER 32
#define WORD_SIZE 8

enum SendState { WAIT, LOAD, COPY, COPY_FROM_MEM, COPY_TO_MEM, FINISH };

enum NocState { HEADER, PAYLOAD, DATA};
enum ArbiterState { ROUND, SEND, RECEIVE };


class DmniModel : public Process {

	private:
	
		//memory to attach the dmni
		MemoryModel* mem; 
		
		//memory mapped 
		MemoryAddr intr; //interruption addr
		MemoryAddr mmr;  //mmr addresss
		
		//I/O buffers
		Buffer* ib;
		Buffer* ob;
		
		//state variables (signals)
		bool read_enable, write_enable, prio;
		bool send_active_2, receive_active_2;
		uint32_t timer;
		
		//internal module states
		ArbiterState arbState;
		SendState sendState, recvState;
		NocState nocState;

	public:  

		/** Implementation of the Process' interface
		  * @return time taken for perming next cycle */
		unsigned long long Run();
		
		/** Ctor.
		  * @param name: name for this instance of Process
		  * @param mem: memory model to attach the dmni to
		  * @param intr: interrupt address
		  * @param mmr: mmr address */
		DmniModel(string name, MemoryModel* m, MemoryAddr intr, MemoryAddr mmr);
	
		/** Dtor. */
		~DmniModel();
		
		//getters and setters for buffers
		void SetOutputBuffer(Buffer* b);
		void SetInputBuffer(Buffer* b);
		Buffer* GetOutputBuffer(Buffer* b);
		Buffer* GetInputBuffer(Buffer* b);
		
		//DMNI specific
		void proc_arbiter();
		void proc_receive();
		void proc_send();
		void proc_config();
		
		//reset
		void reset();
};


#endif /* DMNI */
