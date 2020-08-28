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
#include "Engine.hpp"

using orcasim::ursa::Engine;
using orcasim::ursa::SimulationTime;

/**
 * Defaul constructor
 */
Engine::Engine() {
    _globalTime = 0;
    _epochs = 0;
    _timeout = 1;
}

/**
  * @brief Runs an epoch of simulation
  * @param time Number of cycles to run
  * @return the time in which the last event was executed. Should
  * roughly correspond to <time>. */
SimulationTime Engine::Run(SimulationTime time) {
    _globalTime = 0;
    _timeout = time;

    while (_globalTime <= _timeout) {
        #ifdef URSA_QUEUE_SIZE_CHECKING
        if (_queue.size() <= 0)
            break;
        #endif

        // get next event to be processed. Since we use a priority
        // queue to store events, the event nealy in time will be
        // popped first
        Event e = _queue.top();
        _queue.pop();

        // update global time
        _globalTime = e.time;

        // schedule current event to be executed after a certain number
        // cycles, defined in the correspondent Run method
        e.time += e.timedModel->Run();

        // push event back to the queue
        _queue.push(e);
    }

    // return point in time in which the last event was executed
    return _globalTime;
}

/**
  * @brief Generate the next epoch for a simulation session.
  * @return ? */
SimulationTime Engine::NextEpoch() {
    // get the number of elements scheduled
    int queue_size = static_cast<int>(_queue.size());

    // time that amount of time
    SimulationTime discount = _globalTime - 1;

    // create a new queue and reschedule events
    Event tmp_queue[queue_size];

    // store events in an array so that we can update
    // them without messing up with the priority queue
    for (int i = 0; i < queue_size; i++) {
        tmp_queue[i] = _queue.top();
        tmp_queue[i].time -= discount;

        _queue.pop();
    }

    // put update events back in simulator's queue
    for (int i = 0; i < queue_size; i++)
        _queue.push(tmp_queue[i]);

    // update epochs counters
    _epochs++;

    return _globalTime;
}

SimulationTime Engine::GetGlobalTime() {
    return _globalTime;
}


SimulationTime Engine::GetEpochs() {
    return _epochs;
}

/**
 * @brief Schedule an event to run in a certain period of time
 * @param e Event to run.*/
void Engine::Schedule(const Event& e) {
    #ifdef URSA_ZERO_TIME_CHECKING
    if (e.time == 0) {
        throw std::runtime_error("Simulator: unable to schedule "
            + e.timedModel->GetName() + " to run in the past. " +
            "Events must be scheduled to run with time > 0.");
    }
    #endif
    _queue.push(e);
}

/**
 * @brief Dtor. */
Engine::~Engine() {
    // nothing to do
}
