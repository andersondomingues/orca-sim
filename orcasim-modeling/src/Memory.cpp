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
#include <inttypes.h>

#include <string>
#include <sstream>

#include "Memory.hpp"
#include "Signal.hpp"

using orcasim::modeling::Memory;

Memory::Memory(std::string name, MemoryAddr size, MemoryAddr sram_base,
    bool wipe, std::string binname) : UntimedModel(name) {
    // creates a new array of MemoryType elements (must be disposed by dctor.)
    _mem = new MemoryType[size];

    // set internal fields
    _length = size;
    _base = sram_base;

    // if wipe is true, write zeroes to the whole addressable space
    if (wipe) this->Wipe();

    // if a binary file is provided, load that file into the base address
    if (binname != "")
        this->LoadBin(binname, _base, _length);

    #ifdef MEMORY_ENABLE_COUNTERS
    // if counters are enabled, initiate the respective signals
    // please note that these signals are not mapped to anywhere yet
    _counter_nstore = new Signal<uint32_t>(GetName() + ".counters.store");
    _counter_nload = new Signal<uint32_t>(GetName() + ".counters.load");

    // reset counters (must!)
    _counter_nstore->Write(0);
    _counter_nload->Write(0);
    #endif
}

#ifdef MEMORY_ENABLE_COUNTERS
// getters and setters for the counters' signals
Signal<uint32_t>* Memory::GetSignalCounterStore() {
    return _counter_nstore;
}

Signal<uint32_t>* Memory::GetSignalCounterLoad() {
    return _counter_nload;
}
#endif

void Memory::Write(MemoryAddr addr, MemoryType* data, uint32_t length) {
    #ifdef MEMORY_WRITE_ADDRESS_CHECKING
    // check whether we're trying to access an address
    // lower than the base address
    if (addr < _sram_base) {
        stringstream s;
        s << this->GetName() << ": unable to write to addr (0x" << std::hex
            << addr << ") lower than sram base.";

        throw std::runtime_error(s.str());
    }

    // check whether we're trying to access and address
    // higher than the last address
    if ((addr + length -1) > GetLastAddr()) {
        stringstream s;
        s << this->GetName() << ": unable to write to addr (0x" << std::hex
            << addr << ") higher than last mapped address of (0x" << std::hex
            << GetLastAddr() << ").";
        throw std::runtime_error(s.str());
    }
    #endif

    #ifdef MEMORY_ENABLE_COUNTERS
    // increment number of stores into nstore counter
    _counter_nstore->Inc(1);
    #endif

    // write data to the memory
    // @TODO(ad): check whether this has the best performance
    for (uint32_t i = 0; i < length; i++) {
        _mem[addr - _base] = data[i];
        addr++;
    }
}

void Memory::Read(MemoryAddr addr, MemoryType* buffer, uint32_t length) {
    #ifdef MEMORY_READ_ADDRESS_CHECKING
    // check whether we'are trying to read from an address
    // lower than the base address
    if (addr < _sram_base) {
        stringstream s;
        s << this->GetName() << ": unable to read from addr (0x" << std::hex
            << addr << ") lower than sram base.";
        throw std::runtime_error(s.str());
    }

    // check whether we're trying to read from an address
    // higher than the last address
    if ((addr + length) - 1 > GetLastAddr()) {
        stringstream s;
        s << this->GetName() << ": unable to read from addr (0x" << std::hex
            << addr << ") higher than last mapped address of (0x" << std::hex
            << GetLastAddr() << ").";
        throw std::runtime_error(s.str());
    }
    #endif

    // read data from memory
    // @TODO(ad): check whether this has the best performance
    for (uint32_t i = 0; i < length; i++)
        buffer[i] = _mem[(addr - _base) + i];

    #ifdef MEMORY_ENABLE_COUNTERS
    // increment number of loades into nstore counter
    _counter_nload->Inc(1);
    #endif
}

MemoryType* Memory::GetMap(MemoryAddr addr) {
    #ifdef MEMORY_MAP_ADDRESS_CHECKING
    // check whether we're trying to get the pointer
    // to a cell whose address is lower than the base
    // address
    if (addr < _sram_base) {
        stringstream s;
        s << this->GetName() << ": unable to map from to addr (0x" << std::hex
            << addr << ") lower than sram base.";
        throw std::runtime_error(s.str());
    }

    // check whether we're trying to get the pointer
    // to a cell whose address is higher than the last
    // address
    if (addr > GetLastAddr()) {
        stringstream s;
        s << this->GetName() << ": unable to map from to addr (0x" << std::hex
            << addr << ") higher than last mapped address of (0x" << std::hex
            << GetLastAddr() << ").";
        throw std::runtime_error(s.str());
    }
    #endif

    // return the address to the requested cell
    return &(_mem[addr - _base]);
}

// return the base address (getter)
MemoryAddr Memory::GetBase() {
    return _base;
}

// return the memory size (getter)
uint32_t Memory::GetSize() {
    return _length;
}

// return the address of the last addressable cell
MemoryAddr Memory::GetLastAddr() {
    return (_base + _length) - 1;
}

// wipe without parameters is a shortcut to wipeing
// the whole memory
void Memory::Wipe() {
    this->Wipe(_base, _length);
}

void Memory::Wipe(MemoryAddr base, uint32_t size) {
    #ifdef MEMORY_WRITE_ADDRESS_CHECKING
    // base must be less than the last address
    if (base < _base)
        throw std::runtime_error(this->GetName() +
            ": unable to wipe from base (" + std::to_string(base) +
            ") lower than sram base .");

    // base must be less than the last address
    if (base > GetLastAddr())
        throw std::runtime_error(this->GetName() +
            ": unable to wipe from base (" + std::to_string(base) +
            ") higher than sram base .");

    // base + size cannot go further than the last address
    if (base + size > GetLastAddr())
        throw std::runtime_error(this->GetName() +
            ": unable to wipe that much length (" + std::to_string(size) +
            ") higher than sram base + length.");
    #endif

    // get the pointer to the desired base
    MemoryType* range_ptr = _mem + (base - _base);

    // write zeros to the data to up to <size> cells.
    for (uint32_t i = 0; i < size; i++)
        range_ptr[i] = 0;
}

void Memory::LoadBin(std::string filename, uint32_t base, uint32_t size) {
    // try to open the file
    std::ifstream f(filename, std::ios::binary | std::ios::in | std::ios::out);

    // if we could open the file properly, load it at the base address
    if (f.is_open()) {
        f.read(reinterpret_cast<char*>(&_mem[base -_base]),
            sizeof(_mem[0]) * size);
        f.close();

    // if could not open the file, throw exception
    } else {
        std::string err_msg = this->GetName() + ": unable to load '"
            + filename + "'.";
        throw std::runtime_error(err_msg);
    }
}

void Memory::SaveBin(std::string filename, uint32_t base, uint32_t size) {
    std::ofstream f(filename, std::ifstream::binary);

    if (f.is_open()) {
        f.write(reinterpret_cast<char*>(&_mem[base -_base]),
            sizeof(_mem[0]) * size);
        f.close();
    } else {
        std::string err_msg = this->GetName() +
            ": unable to save'" + filename + "'.";
        throw std::runtime_error(err_msg);
    }
}

// Dump is a shorthand for Dump(x, y)
void Memory::Dump() {
    this->Dump(0, _length);
}

void Memory::Dump(uint32_t base, uint32_t length) {
    uint32_t k, l;

    // mask is necessary to correct a bug(?) when printing
    // negative hexas.
    uint32_t mask = 0x000000FF;
    int8_t ch;

    // print in lines containing 16 bytes
    for (k = 0; k < length; k += 16) {
        // print the address
        printf("\n%08x ", base + k);
        for (l = 0; l < 16; l++) {
            printf("%02x ", _mem[k + l + (base - _base)] & mask);
            if (l == 7) putchar(' ');
        }

        // print the data in ascii ("." is printed when no ascii char exists)
        printf(" |");
        for (l = 0; l < 16; l++) {
            ch = _mem[k + l + (base - _base)];
            if ((ch >= 32) && (ch <= 126))
                putchar(ch);
            else
                putchar('.');
        }
        putchar('|');
    }
}

Memory::~Memory() {
    // delete all cells
    delete [] _mem;

    #ifdef MEMORY_ENABLE_COUNTERS
    // delete signals that hold counters
    delete(_counter_nstore);
    delete(_counter_nload);
    #endif
}
