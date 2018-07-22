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

#define TAM_BUFFER_DMNI 32 
#define DMNI_TIMER 32
#define WORD_SIZE 8

#define BUFFERR_LENGTH 32
#define IS_HEADER_LENGTH 32

enum class SendState { 
    WAIT, LOAD, COPY, COPY_FROM_MEM, COPY_TO_MEM, FINISH };
enum class RecvState {
    WAIT, COPY_TO_MEM, FINISH }; 
enum class NocState { 
    HEADER, PAYLOAD, DATA};
enum class ArbiterState { 
    ROUND, SEND, RECEIVE };

class DmniModel : public Process {

	private:
	
		//memory to attach the dmni to
		MemoryModel* mem; 
		
		//memory mapped 
		MemoryAddr intr; //interruption addr
		MemoryAddr mmr;  //mmr addresss
		
		//I/O buffers
		Buffer* ib;
		Buffer* ob;
		
        //internal module states
		ArbiterState arbState;
		SendState sendState;
        RecvState recvState;
		NocState nocState;
        
        
        //state variables (signals)
        bool read_enable, write_enable, prio;
		bool send_active_2;
        uint32_t send_address, send_address_2, send_size, send_size_2;
        
        bool read_active_2;
        bool recv_active_2;
        uint32_t recv_address, recv_size;
		uint32_t timer;
		
        bool rx, tx;
        
		uint32_t first, last, payload_size, size, address, intr_counter_temp;
        
        uint32_t data_in, data_out;
        
        uint32_t bufferr[BUFFERR_LENGTH], mem_data_write;
        
        bool add_buffer, read_av, start, operation, slot_available;
        uint8_t mem_byte_we; // "XXXX"
        
        bool is_header[IS_HEADER_LENGTH]; 
        

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
