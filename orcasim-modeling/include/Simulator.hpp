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

#include <vector>
#include <string>

#include "TimedModel.hpp"
#include "UntimedModel.hpp"
#include "Engine.hpp"
#include "Signal.hpp"
#include "SimulationTime.hpp"

using orcasim::base::TimedModel;
using orcasim::base::UntimedModel;
using orcasim::base::Engine;
using orcasim::base::SimulationTime;
using orcasim::modeling::Signal;


namespace orcasim::modeling {

static void sig_handler(int _);  // interruption handler

enum class SimulatorInterruptionStatus {
    RUNNING,      // application is running
    INTERRUPTED,  // app have been interrupted once
    ABORTED       // app have been interrupted twice, aborting
};

class Simulator {
 private:
    std::chrono::high_resolution_clock::time_point t1, t2;  // time measurement
    std::vector<std::string> _params;  // argc+argv

    Engine _engine;  // the simulation engine

    int _exit_status;

    static volatile SimulatorInterruptionStatus _interruption_status;

 public:
    static void SetInterruptionStatus(SimulatorInterruptionStatus status);
    static SimulatorInterruptionStatus GetInterruptionStatus();

    Simulator(int argc, char** argv);
    Simulator();

    void virtual Startup() = 0;  // model instantiation
    void virtual Schedule() = 0;
    void virtual Simulate();  // simulation
    void virtual Report() = 0;   // statistics

    void Register(TimedModel* m);
    void Register(TimedModel* m, SimulationTime t);

    int GetExitStatus();
    void SetExitStatus(int status);

    std::string GetParam(int index);
};

}  // namespace orcasim::modeling
#endif  // ORCASIM_MODELING_INCLUDE_SIMULATOR_HPP_
