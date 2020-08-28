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
#ifndef ORCASIM_BASE_INCLUDE_EVENT_HPP_
#define ORCASIM_BASE_INCLUDE_EVENT_HPP_

#include "TimedModel.hpp"
#include "SimulationTime.hpp"

namespace orcasim::base {

/**
 * This class models a discrete event. In orca-sim, events have a reference
 * (a pointer) to the <cycle> funcion of the associated hardware model, which
 * will be executed at time <time> by the simulation engine. Events occur once.
 */
class Event{
 public:
    /**
     * Point in time when the event will trigger. 
     * */
    SimulationTime time;

    /**
     * Model whose activation function will be called 
     * once the event triggers.
     */
    TimedModel* timedModel;

    Event(SimulationTime, TimedModel*);
    Event();  // necessary for arrays

    /**
     * Comparing operator. An event occurs first in time if its time 
     * is less than the compared event (required by the priority queue, see
     * <std::priority_queue>).
     */
    bool operator<(const Event& e) const;
};

}  // namespace orcasim::base
#endif  // ORCASIM_BASE_INCLUDE_EVENT_HPP_
