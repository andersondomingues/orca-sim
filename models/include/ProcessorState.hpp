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
#ifndef MODELS_INCLUDE_PROCESSORSTATE_HPP_
#define MODELS_INCLUDE_PROCESSORSTATE_HPP_

#define NUMBER_OF_REGISTERS 32

#include <stdint.h>

/** Defines a generic state model 
  * for use within processor models. */
template <typename T>
struct ProcessorState{
    // please note that register to from 0 to NUM_REGS, and
    // the next reg (which would overflow the array size)
    // falls in the PC instead. For example, for an arch with
    // 32 register, the register number 33 is PC.
    T regs[NUMBER_OF_REGISTERS];  // general purpose registers
    T pc;       // the value of pc in the current cycle
    T pc_prev;  // value of pc in the last cycle (zero if 1st)
    T pc_next;  // value of pc in the next cycle

    #ifdef ORCA_ENABLE_GDBRSP
    T bp;       // flag's risen if breakpoint has being reached
    T pause;    // indicates that CPU is paused (works as a second stall line)
    T steps;    // number of remaining instructions until next pause
    #endif

    T terminated;  // indicate whether the cpu has aborted (abnormaly or not)
};

// Some of the most used instances. More can be added later.
template struct ProcessorState<uint8_t>;
template struct ProcessorState<uint16_t>;
template struct ProcessorState<uint32_t>;
template struct ProcessorState<uint64_t>;

// Some of the most used instances. More can be added later.
template struct ProcessorState<int8_t>;
template struct ProcessorState<int16_t>;
template struct ProcessorState<int32_t>;
template struct ProcessorState<int64_t>;

#endif  // MODELS_INCLUDE_PROCESSORSTATE_HPP_
