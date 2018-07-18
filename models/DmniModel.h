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

#include <iostream>
#include <Process.h>

#define TAM_BUFFER_DMNI 
#define DMNI_TIMER 32
#define WORD_SIZE 8

enum SendState { WAIT, LOAD, COPY, COPY_FROM_MEM, COPY_TO_MEM, FINISH };

enum NocState { HEADER, PAYLOAD, DATA};
enum ArbiterState { ROUND, SEND, RECEIVE };

typedef struct{
	
	signal bufferr: buff_dmni := (others=>(others=>'0'));
	subtype buffsizebool is std_logic_vector(0 to (TAM_BUFFER_DMNI-1)); 

	signal is_header: buffsizebool := (others=> '0');
	signal first,last: pointer := (others=>'0');
	signal payload_size      : regflit;


	uint32_t timer, intr_count, intr_counter_temp;

	uint32_t address, address_2, size, size_2, send_address, send_address_2,
		send_size, send_size_2, recv_address, recv_size;

	//0 - copy from memory
	//1 - copy to memory
	bool operation;
	
	bool prio, read_av, slot_available, add_buffer;

	bool write_enable, read_enable;
	bool send_active_2, receive_active_2;

} DmniState;

class DmniModel : public Process {

	private:
		//internal state of modules
		SendState sendState, recvState;
		
		NocState nocState;
		ArbiterState arbState;
		
		//config interface
		bool set_address, set_address_2;
		bool set_size, set_size_2;
		bool set_op, start, 
		
		uint32_t config_data;
		
		//status
		bool intr, send_active, receive_active;
		
		DmniState s;
		
	public:  

		//Process impl
		unsigned long long Run();
		DmniModel(string name);
		~DmniModel();
		
		//DMNI specific
		void proc_arbiter();
		void proc_receive();
		void proc_send();
		void proc_config();
};


#endif /* DMNI */
