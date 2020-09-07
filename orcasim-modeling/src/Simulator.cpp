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
#include <signal.h>

#include <chrono>

#include "Simulator.hpp"
#include "Event.hpp"

#define ORCA_EPOCH_LENGTH 100000
#define ORCA_EPOCHS_TO_SIM 10000


using orcasim::base::Event;
using orcasim::modeling::Simulator;
using orcasim::modeling::SimulatorInterruptionStatus;

static void orcasim::modeling::sig_handler(int _) {
    (void)_;

    switch (Simulator::GetInterruptionStatus()) {
    case SimulatorInterruptionStatus::RUNNING:
        Simulator::SetInterruptionStatus(
            SimulatorInterruptionStatus::INTERRUPTED);
        std::cout << std::endl
            << "Simulation interrupted. Wait for the current epoch to finish "
            << "or press CTRL+C again to force quit." << std::endl;
        break;
    case SimulatorInterruptionStatus::INTERRUPTED:
        Simulator::SetInterruptionStatus(SimulatorInterruptionStatus::ABORTED);
        exit(0);
        break;
    }
}

volatile SimulatorInterruptionStatus Simulator::_interruption_status
    = SimulatorInterruptionStatus::RUNNING;

void Simulator::SetInterruptionStatus(SimulatorInterruptionStatus status) {
    Simulator::_interruption_status = status;
}

SimulatorInterruptionStatus Simulator::GetInterruptionStatus() {
    return Simulator::_interruption_status;
}

std::string Simulator::GetParam(int index) {
    return _params[index];
}

Simulator::Simulator(int argc, char** argv) {
    _exit_status = 0;  // if not overwritten, exist status is zero (EXIT_OK)
    _interruption_status = SimulatorInterruptionStatus::RUNNING;
    signal(SIGINT, sig_handler);  // register interruption handler

    // parse params
    if (argc > 0) {
        _params = std::vector<std::string>();
        for (int i = 0; i < argc; i++) {
            char* param = argv[i];
            _params.push_back(std::string(param));
        }
    }
}

void Simulator::Register(TimedModel* model, SimulationTime time) {
    _engine.Schedule(Event(time, model));
}

void Simulator::Register(TimedModel* model) {
    Simulator::Register(model, 1);
}

void Simulator::Simulate() {
    try {
        while (_interruption_status == SimulatorInterruptionStatus::RUNNING) {
            t1 = std::chrono::high_resolution_clock::now();

            _engine.Run(ORCA_EPOCH_LENGTH);

            t2 = std::chrono::high_resolution_clock::now();

            auto duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                    .count();
            _engine.NextEpoch();

            // converts mili to seconds before calculating the frequency
            double hertz = static_cast<double>(ORCA_EPOCH_LENGTH) /
                (static_cast<double>(duration) / 1000.0);

            // divide frequency by 1k (Hz -> KHz)
            std::cout << "notice: epoch #" << _engine.GetEpochs() << " took ~"
                << duration << "ms (running @ " << (hertz / 1000000.0)
                << " MHz)" << std::endl;

            #ifdef ORCA_EPOCHS_TO_SIM
            // simulate until reach the limit of pulses
            if (_engine.GetEpochs() >= ORCA_EPOCHS_TO_SIM)
                break;
            #endif
        }
    } catch(std::runtime_error& e) {
        std::cout << e.what() << std::endl;
        _exit_status = -1;  // abnormal termination, simulation failed
    }

    Report();
}

int Simulator::GetExitStatus() {
    return _exit_status;
}

void Simulator::SetExitStatus(int status) {
    _exit_status = status;
}
