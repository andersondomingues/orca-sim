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

//api includes
#include <UntimedModel.h>

#include <UMau.h>

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


UMau::UMau(std::string name = "", bool allow_overleap = false){
    _name = name;
    _allow_overleap = allow_overleap; //permits to mapping various virt. addr. to the same phys. addr.
}
   
void UMau::Map(UMemory* mem, uint32_t base, uint32_t length){
    
    MemoryMapping m = MemoryMapping(
        base, length, (length - base - 1), mem, UMEMORY);

    this->add_mapping(m); //use list<> form STL
}

void UMau::Map(Comm<uint8_t>*  comm, uint32_t base){

    MemoryMapping m = MemoryMapping(
        base, 1, base, comm, UCOMM);

    this->add_mapping(m); //see note above
}


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
