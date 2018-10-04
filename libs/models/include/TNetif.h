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
#ifndef __TNETIF_H
#define __TNETIF_H

//std API
#include <iostream>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>
#include <UMemory.h>
#include <UComm.h>

typedef uint16_t FlitType;

enum class NIRecvState{ IDLE, WRITE_DATA, RECV_TO_MEM, INTR_CPU};
enum class NISendState{ IDLE, SETUP, SEND, DONE };

/**
 * @class TNetif
 * @author Anderson Domingues
 * @date 10/03/18
 * @file TNetif.h
 * @brief This class models the behaviour of a network
 * interface based on the one from HermesSRC platform
 */
class TNetif: public TimedModel{

private:
		
        UBuffer<FlitType>* _ob;  //output buffers
        UBuffer<FlitType>* _ib;  //input buffers
		UMemory* _mem;           //memory ptr
		
		uint32_t _base_addr; //memory-mapped buffer
		
		
		//internal state
		NIRecvState _recv_state;
		NISendState _send_state;
		
		//recv proc vars
		uint32_t _words_to_send; //to router
		uint32_t _words_to_copy; //to memory
		
		
		uint32_t _next_addr;
				
		//EXTERNAL INTERFACE
		UComm<uint8_t>* _send_intr;
		UComm<uint8_t>* _recv_intr;
		
		bool _intr;
public: 

		/** buffers **/
		
		//buffer that outputs to the router
        void SetOutputBuffer(UBuffer<FlitType>* b);
		UBuffer<FlitType>* GetOutputBuffer();
		
		//buffer which comes from the dma
		UBuffer<FlitType>* GetInputBuffer();
		
		
		/** internal processes */
		void sendProcess();
		void recvProcess();
		
		void SetBaseAddr(uint32_t addr);
				
		/** others **/
		unsigned long long Run();
		void Reset();

        /** ctor. **/
		TNetif(string name, uint32_t addr = 0);
	
		/** dtor. **/
		~TNetif();
};


#endif /* TNETIF_H */
