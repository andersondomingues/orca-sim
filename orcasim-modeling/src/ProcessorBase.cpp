/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software
 * http://https://github.com/andersondomingues/orca-mpsoc
 * http://https://github.com/andersondomingues/orca-tools
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
#include "TimedModel.hpp"
#include "ProcessorBase.hpp"

#ifdef ORCA_ENABLE_GDBRSP
template <typename T>
uint32_t TProcessorBase<T>::GDBSERVER_PORT = ORCA_GDBRSP_PORT;
#endif

using orcasim::modeling::ProcessorBase;
using orcasim::modeling::ProcessorState;
using orcasim::modeling::Memory;
using orcasim::base::SimulationTime;
using orcasim::base::TimedModel;

template <typename T>
ProcessorBase<T>::ProcessorBase(std::string name,
    MemoryAddr initial_pc, Memory* mem) : TimedModel(name) {

    // set mem ptr
    _memory = mem;

    // reset registers
    for (int i = 0; i < NUMBER_OF_REGISTERS; i++)
        _state.regs[i] = 0;

    // reset PC
    _state.pc_prev = initial_pc;
    _state.pc = initial_pc;
    _state.pc_next = _state.pc + sizeof(T);

    // reset flags
    #ifdef ORCA_ENABLE_GDBRSP
    _state.bp = 0;  // no breakpoint reached yet
    _state.pause = 1;  // starts paused in gdb mode
    _state.steps = 0;  // no steps to be performed, wait for gdb
    _gdbserver = new RspServer<T>(&_state,
        _memory, "127.0.0.1", GDBSERVER_PORT++);
    #endif

    // reset special flags
    _state.terminated = false;
}

template <typename T>
ProcessorBase<T>::~ProcessorBase() {
    #ifdef ORCA_ENABLE_GDBRSP
    delete _gdbserver;
    #endif
}

/**
 * Access the current state of the processor.
 * @return A pointer to the state of the processor.
 */
template <typename T>
inline ProcessorState<T>* ProcessorBase<T>::GetState() {
    return &_state;
}

template <typename T>
inline Memory* ProcessorBase<T>::GetMemory() {
    return _memory;
}

template <typename T>
SimulationTime ProcessorBase<T>::Run() {
    return 0;
    // nothing todo
}