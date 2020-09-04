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
#ifndef ORCASIM_GDBRSP_INCLUDE_GDBPROCESSORBASE_HPP_
#define ORCASIM_GDBRSP_INCLUDE_GDBPROCESSORBASE_HPP_

// base API includes
#include <iostream>
#include <list>
#include <string>

#include "ProcessorBase.hpp"
#include "SimulationTime.hpp"
#include "GdbProcessorState.hpp"
#include "Memory.hpp"
#include "RspServer.hpp"

using orcasim::base::SimulationTime;
using orcasim::modeling::Memory;
using orcasim::modeling::ProcessorBase;
using orcasim::gdbrsp::GdbProcessorState;
using orcasim::gdbrsp::RspServer;

namespace orcasim::gdbrsp {

template <typename T>
class GdbProcessorBase : public ProcessorBase<T>{
 private:
    RspServer<T>* _gdbserver;
    GdbProcessorState<T>* _state;

 public:
    static uint32_t GDBSERVER_PORT;
    GdbProcessorBase(std::string name, MemoryAddr initial_pc,
        Memory* mem, std::string gdb_ip, int gdb_prt);

    SimulationTime Run();
    GdbProcessorState<T>* GetState();
};

// Some of the most used instances. More can be added later.
template class GdbProcessorBase<uint8_t>;
template class GdbProcessorBase<uint16_t>;
template class GdbProcessorBase<uint32_t>;
template class GdbProcessorBase<uint64_t>;

// Some of the most used instances. More can be added later.
template class GdbProcessorBase<int8_t>;
template class GdbProcessorBase<int16_t>;
template class GdbProcessorBase<int32_t>;
template class GdbProcessorBase<int64_t>;

}  // namespace orcasim::gdbrsp
#endif  // ORCASIM_GDBRSP_INCLUDE_GDBPROCESSORBASE_HPP_

