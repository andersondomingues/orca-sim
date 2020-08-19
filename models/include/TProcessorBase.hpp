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
#ifndef MODELS_INCLUDE_TPROCESSORBASE_HPP_
#define MODELS_INCLUDE_TPROCESSORBASE_HPP_

#include <string>

#ifdef ORCA_ENABLE_GDBRSP
#include "RspServer.hpp"
#endif

#include "TimedModel.hpp"
#include "UMemory.hpp"
#include "ProcessorState.hpp"

/** This class implements the base operation for 
 * generic processor implementations. This class
 * uses templates to handle the size of internal
 * registers (registers are limited to have the 
 * same size) */
template <typename T>
class TProcessorBase : public TimedModel{
 private:
    /** struct that represent the state of a processor. 
     * All processor share the same state model, which 
     * permit us to implement features such as the GDB's
     * remote debug server. */
    struct ProcessorState<T> _state;

    /** We assume that every processor is attached to a 
     * memory core. For now, the only core available is 
     * untimed. */
    UMemory* _memory;

    #ifdef ORCA_ENABLE_GDBRSP
    RspServer<T>* _gdbserver;
    static uint32_t GDBSERVER_PORT;
    char _buffer[2000];
    #endif

 public:
    /** Default constructor.
     * @param name A short name or description of the instance
     * @param initial_pc Memory address to which the PC
        register will point to at the startup. */
    TProcessorBase(std::string name, MemoryAddr initial_pc, UMemory* mem);

    /** Destructor. */
    ~TProcessorBase();

    /** Run method from the base TimedModel class, overloaded. 
     * We include in the overloading external components that 
     * would apply to all processors. Examples include energy 
     * estimation (through counters) and GDBRSP. 
     * @returns the number of cycles to skip until next schedule. */
    SimulationTime Run();

    /** This method returns the state model of the processor. This
     * is ideally used from the top level simulator to report 
     * processor states at the end of simulation.
     * @returns a pointer to the processor state struct. **/
    ProcessorState<T>* GetState();

    /** This method returns a pointer to the object that models 
     * the memory core. It is made private to avoid being changed 
     * by the processor core implementation.
     * @returns a pointers to the memory model */
    UMemory* GetMemory();
};

// Some of the most used instances. More can be added later.
template class TProcessorBase<uint8_t>;
template class TProcessorBase<uint16_t>;
template class TProcessorBase<uint32_t>;
template class TProcessorBase<uint64_t>;

// Some of the most used instances. More can be added later.
template class TProcessorBase<int8_t>;
template class TProcessorBase<int16_t>;
template class TProcessorBase<int32_t>;
template class TProcessorBase<int64_t>;

#endif  // MODELS_INCLUDE_TPROCESSORBASE_HPP_
