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
#ifndef URSA_INCLUDE_ENGINE_HPP_
#define URSA_INCLUDE_ENGINE_HPP_

// lib dependencies
#include <iostream>
#include <array>
#include <queue>
#include <string>

// own api dependencies
#include "Event.hpp"
#include "SimulationTime.hpp"

namespace orcasim::ursa {

/**
 * This class implements an event queue to schedule and 
 * execute hardware modules. The <priority_queue> from 
 * the std lib is used to sort elements by time.
 */
class Engine{
 private:
    /** number of cycles to simulate before reseting the queue */
    SimulationTime _epochs;

    /** queue that stores all events */
    std::priority_queue<Event> _queue;

    /** The global clock, stores current simulation time. */
    SimulationTime _globalTime;

    /** max time the simulation can reach */
    SimulationTime _timeout;

    /** execute event at top of event queue */
    void executeNext();

 public:
    /**
     * Default constructor, takes no parameters. 
     */
    Engine();

    /* run the simulation for <time> cycles. */
    /**
     * Executes the simulator until the internal clock reaches <time> cycles.
     * @param time Maximum cycles to simulate
     * @return the time in which the clock ended the simulation.
     */
    SimulationTime Run(SimulationTime time = 100000);

    /**
     * Gets the current global time.
     * @return value of _globalTime
     */
    SimulationTime GetGlobalTime();

    /**
     * Gets the number of cycles to simulate before reseting
     * the simulation clock.
     * @return value of __epochs 
     */
    SimulationTime GetEpochs();

    /**
     * Resets the simulation clock and advance simulation to the next epoch.
     * @return The time in which the simulation clock was before advancing
     *     to the next epoch.
     */
    SimulationTime NextEpoch();

    /**
     * Adds an event to the simulation queue.
     * @param e Event to be added
     */
    void Schedule(const Event& e);

    /**
     * Destructor.
     */
    ~Engine();
};

}  // namespace orcasim::ursa
#endif  // URSA_INCLUDE_ENGINE_HPP_
