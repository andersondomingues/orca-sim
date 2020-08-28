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
#ifndef URSA_INCLUDE_TIMEDMODEL_HPP_
#define URSA_INCLUDE_TIMEDMODEL_HPP_

#include <string>

// own api includes
#include "Model.hpp"
#include "SimulationTime.hpp"

namespace orcasim::ursa {

/**
 * This class models a TimedModel. In this project, a TimedModel
 * is an abstraction which can execute an action in a given point
 * in time. For example, hardware can be modeled as TimedModeles that 
 * execute cycles given some period. */
class TimedModel : public Model{
 public:
    /** Default Ctor. */
    explicit TimedModel(std::string name);

    /**
     * Method which is called by the simulator when during the 
     * execution of the TimedModel. Must be implemented by subclasses.*/
    virtual SimulationTime Run() = 0;

    /**
     * Dtor. Must be implemented by subclasses. */
    virtual ~TimedModel() = 0;

    /**
     * Resets the instance to its starting state. Must be implemented
     * by subclasses */
    virtual void Reset() = 0;
};

}  // namespace orcasim::ursa
#endif  // URSA_INCLUDE_TIMEDMODEL_HPP_
