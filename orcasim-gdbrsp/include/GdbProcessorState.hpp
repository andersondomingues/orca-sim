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
#ifndef ORCASIM_GDBRSP_INCLUDE_GDBPROCESSORSTATE_HPP_
#define ORCASIM_GDBRSP_INCLUDE_GDBPROCESSORSTATE_HPP_

#define NUMBER_OF_REGISTERS 32

#include <stdint.h>
#include "ProcessorState.hpp"

using orcasim::modeling::ProcessorState;

namespace orcasim::gdbrsp {

/** Defines a generic state model 
  * for use within processor models. */
template <typename T>
struct GdbProcessorState : ProcessorState<T> {
    T bp;
    T pause;
    T steps;
};

// Some of the most used instances. More can be added later.
template struct GdbProcessorState<uint8_t>;
template struct GdbProcessorState<uint16_t>;
template struct GdbProcessorState<uint32_t>;
template struct GdbProcessorState<uint64_t>;

// Some of the most used instances. More can be added later.
template struct GdbProcessorState<int8_t>;
template struct GdbProcessorState<int16_t>;
template struct GdbProcessorState<int32_t>;
template struct GdbProcessorState<int64_t>;

}  // namespace orcasim::gdbrsp
#endif  // ORCASIM_GDBRSP_INCLUDE_GDBPROCESSORSTATE_HPP_
