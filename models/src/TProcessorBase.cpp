/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software-tools
 * http://https://github.com/andersondomingues/orca-mpsoc
 *
 * Copyright (C) 2018-2020 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. 
******************************************************************************/
#include <TimedModel.h>
#include <TProcessorBase.h>

#ifdef ORCA_ENABLE_GDBRSP
template <typename T> 
uint32_t TProcessorBase<T>::GDBSERVER_PORT = ORCA_GDBRSP_PORT;
#endif

template <typename T> 
TProcessorBase<T>::TProcessorBase(std::string name, MemoryAddr initial_pc, UMemory* mem) : TimedModel(name) {

	//set mem ptr
	_memory = mem;

	//reset registers
	for(int i = 0; i < NUMBER_OF_REGISTERS; i++)
		_state.regs[i] = 0;

	//reset PC
	_state.pc = initial_pc;
	_state.pc_next = _state.pc + sizeof(T);

	#ifdef ORCA_ENABLE_GDBRSP
	_gdbserver = new RspServer<T>(&_state, _memory, "127.0.0.1", GDBSERVER_PORT++);
	#endif
}

template <typename T> 
TProcessorBase<T>::~TProcessorBase(){
	#ifdef ORCA_ENABLE_GDBRSP
	delete _gdbserver;
	#endif
}

/**
 * Access the current state of the processor.
 * @return A pointer to the state of the processor.
 */
template <typename T> 
inline ProcessorState<T>* TProcessorBase<T>::GetState(){
	return &_state;
}

template <typename T>
inline UMemory* TProcessorBase<T>::GetMemory(){
	return _memory;
}

/**
 * Run method. Processors must either (i) run one 
 * instruction and return the number of cycles that 
 * the instruction took to leave the pipeline, or
 * (ii) execute one cycle and return 1.
 */
template <typename T>
SimulationTime TProcessorBase<T>::Run(){

	#ifdef ORCA_ENABLE_GDBRSP
	return _gdbserver->Receive();
	#else
	return 0;
	#endif
}
