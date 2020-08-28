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
#ifndef ORCASIM_MODELING_INCLUDE_SIMULATOR_HPP_
#define ORCASIM_MODELING_INCLUDE_SIMULATOR_HPP_

#include <list>

#include "TimedModel.hpp"
#include "Engine.hpp"

using orcasim::base::TimedModel;
using orcasim::base::Engine;
using orcasim::base::SimulationTime;

namespace orcasim::modeling {

class Simulator {
 private:
    std::list<TimedModel> _models;
    Engine _engine;

 public:
    void Startup();  // model instantiation
    void Simulate();  // simulation
    void Report();   // statistics
    void Cleanup();  // delete instances

    void Register(TimedModel* m);
    void Register(TimedModel* m, SimulationTime t);
    void Unregister(TimedModel* m);
};

}  // namespace orcasim::modeling
#endif  // ORCASIM_MODELING_INCLUDE_SIMULATOR_HPP_
