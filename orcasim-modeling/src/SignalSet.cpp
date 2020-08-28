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

#include <stdint.h>

// lib dependent includes
#include <iostream>
#include <vector>
#include <algorithm>


// api includes
#include "UntimedModel.hpp"
#include "Signal.hpp"
#include "SignalSet.hpp"
#include "Memory.hpp"

using orcasim::modeling::SignalSet;
using orcasim::modeling::Signal;

// ctor.
template <typename T>
SignalSet<T>::SignalSet(std::string name, uint32_t nsig): UntimedModel(name) {
    // set internal variables
    _num_signals = nsig;
    _signals = new Signal<T>*[_num_signals];

    // create a new vector of signals. Signals are no mapped yep, use MapTo.
    for (uint32_t i = 0; i < _num_signals; i++)
        _signals[i] = new Signal<T>(this->GetName() + "." + std::to_string(i));
}

// dtor.
template <typename T>
SignalSet<T>::~SignalSet() {
    delete[] _signals;
}

// mapping function
template <typename T>
void SignalSet<T>::MapTo(MemoryType* memptr, MemoryAddr addr) {
    MemoryAddr address = addr;
    MemoryType* memtype = memptr;

    // set the proper address to each signal and map
    for (uint32_t i = 0; i < _num_signals; i++) {
        _signals[i]->SetAddress(address);
        _signals[i]->MapTo(memtype, address, false);

        address += sizeof(T);

        // caution, pointer arithmetic here
        for (uint32_t j = 0; j < sizeof(T) / sizeof(MemoryType); j++)
            memtype++;
    }
}

// getters
template <typename T>
Signal<T>* SignalSet<T>::GetSignal(uint32_t index) {
    if (index > _num_signals - 1 || index < 0) {
        std::cout << "warn: requested signals is out of the bounds of the set"
            << std::endl;
        return nullptr;
    }

    return _signals[index];
}
