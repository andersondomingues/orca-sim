/** 
 * This file is part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
 *
 * Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. **/
#ifndef __UMAU_H
#define __UMAU_H

//lib dependent includes
#include <iostream>
#include <queue>
#include <stdint.h>

//api includes
#include <UntimedModel.h>

/**
 * @enum MemoryMappingType
 * @brief Enumerates the possible types of memory mapping entries
 * within a Mau object.
 */
enum MemoryMappingType{
    UMEMORY   = 0, //when Map(UMemory* is used...
    UCOMM     = 1  //when Map(UComm* is used...
};

/**
 * @struct MemoryMapping
 * @brief Represents an entry of memory mapping within a Mau object
 */
typedef struct{
    uint32_t base, //starting addres of mapping
    uint32_t len,  //length of memory mapping
    uint32_t end,  //last addres of memory (calculated, for performance purpose)
    void* modptr,  //ptr to the module which holds the data (phys module)
    MemoryMappingType type //type of mapping (comm, umem, etc.)
} MemoryMapping;


/**
 * @class UMau (memory access unit)
 * @author Anderson Domingues
 * @date 11/06/18
 * @file UMau.h
 * @brief This class implements a generic mau (memory access unit). This unit would hold
 * all memory mapping within the system. It serves as a facade to the memory modules, which 
 * MAY NOT be accessed directly from other components.
 **/
template <typename T>
class UMau : public UntimedModel{

private:
    std::string _name;

public:
    /**
     * @brief Constructor. 
     * @param name (optional) An arbitrary name for the instance of Mau.
     * @param allow_overleaps Permite many ranges to point to the same physical addr.
     */
    UMau(std::string name = "", bool allow_mapping_overleap = false);
    
    /**
     * @brief Maps access to a memory module
     * @param mem Module to be mapped
     * @param base Starting address to map
     * @param length Size (in bytes) of the range to be mapped
     */ 
    void Map(UMemory* mem, uint32_t base, uint32_t length);
    
    /**
     * @brief Maps a comm to an address. Supports 1-byte sized comms.
     * @param comm Comm to be mapped
     * @param base Address to map the comm to
     */
    void Map(Comm<uint8_t>*  comm, uint32_t base);

    /**
     * @brief Maps a comm to an address. Supports 2-bytes sized comms.
     * @param comm Comm to be mapped
     * @param base Address to map the comm to
     */
    void Map(Comm<uint16_t>* comm, uint32_t base);

    /**
     * @bried Maps a comm to an address. Supports 4-byte sized comms.
     * @param comm Comm to be mapped
     * @param base Address to map the comm to
     */
    void Map(Comm<uint32_t>* comm, uint32_t base);

    /**
     * @brief Maps a comm to an address. Supports 8-byte sized comms.
     * @param comm Comm to be mapped
     * @param base Address to map the comm to
     */
    void Map(Coom<uint64_t>* coom, uint32_t base);

    /**
     * @brief Destructor. Cleans dynamic allocated memory before disposing the object.
     */
    ~UMau();

    /**
     * @brief Clear all mappings. No use for now.
     */
    void Reset();

    /**
     * @brief Reads data from the virtual memory
     * @param base Starting address of reading
     * @param len Size of data to be read (in bytes)
     * @param out Memory space to write the read values
     */
    void Read(uint32_t base, uint32_t len, int8_t* out);

    /**
     * @brief Writes data to the virtual memory
     * @param base Starting address to write to
     * @param len Size of data to be write
     * @param in Address of data to be copied into virtual memory
     */
    void Write(uint32_t base, uint32_t len, int8_t* in);
};

//Some of the most used instances. More can be added later.
template class UBuffer<uint8_t>;  //mem word
template class UBuffer<uint16_t>; //dmni/noc word
template class UBuffer<uint32_t>; //proc word

#endif /* UBuffer_H */
