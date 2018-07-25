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
#ifndef __NOC_ROUTER_MODEL_H
#define __NOC_ROUTER_MODEL_H

//std API
#include <iostream>

//simulator API
#include <Process.h>
#include <Buffer.h>

//model API
#include <MemoryModel.h>

#define TAM_BUFFER_DMNI 32 
#define DMNI_TIMER 32  /*std_logic_vector(4 downto 0):="10000"*/
#define WORD_SIZE  4   /*std_logic_vector(4 downto 0):="00100"*/

/*
 * The RegFlit type affects both routers from Hermes and 
 * the DMNI modules. Before changing it, make sure that 
 * the change will correcly propagate to all involved 
 * modules*/
typedef uint32_t RegFlit;



class NocRouterModel: public Process {

private:
        
public: 
        void PortMap();
        
		/** Implementation of the Process' interface
		  * @return time taken for perming next cycle */
		unsigned long long Run();
		
		
		NocRouterModel(string name);
	
		/** Dtor. */
		~NocRouterModel();
		
		void Reset();
};


#endif /* DMNI */
