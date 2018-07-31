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
#include <Event.h>
#include <Simulator.h>
#include <MemoryModel.h>
#include <HFRiscvModel.h>
#include <NocRouterModel.h>
#include <DmniModel.h>

#define CYCLES_TO_SIM 100000
#define MEM_SIZE  0x00100000


//this program tests the DmniModel class 
int main(int argc, char** argv){
    
	cout << "Simulation stated" << endl;
	
    //bind DMNI wires to variables
    bool _intr = false;
    uint32_t _addr, _length, _op = OP_NONE;
    
    //instatiates a new memory module 
    MemoryModel mem = MemoryModel("mem1", MEM_SIZE, true);
    mem.Reset();
    
    //instantiates a new dmni module
    DmniModel dmni = DmniModel("dmni1");
    dmni.SetIntr(&_intr);
    dmni.SetAddress(&_addr);
    dmni.SetInputBuffer(new Buffer<FlitType>("dmnibuf"));
    dmni.SetLength(&_length);
    dmni.SetMemoryModel(&mem);
    dmni.SetOperation(&_op);
    dmni.Reset();
    
   	//creates a new simulator and register modules
	Simulator s = Simulator(CYCLES_TO_SIM);
    s.Schedule(Event(0, &mem));
    s.Schedule(Event(0, &dmni));
    
    
    //----- TEST CASE 1: read from memory and write into the output buffer 
    _addr = 0x20;  //arbitrary memory address
    _op = OP_SEND; //configure dmni to send intead of receiving
    _length = 8;   //read 4 words from _addr and put then into output buffer
    
    uint16_t _data[8] = {22, 23, 24, 25, 36, 47, 67, 99}; //dummy data to be sent
    mem.Write(_addr, (int8_t*)_data, 16);  //write 8 words (16x2 bytes) to the memory
    
    //buffer is empty at startup
    std::cout << "output buffer size: " << dmni.GetOutputBuffer()->size() << std::endl;

    int8_t _d2[16];
    mem.Read(0x20, _d2, 16);
    
    for(int i = 0; i < 16; i+=2) std::cout << (int16_t)_d2[i] << " ";
    std::cout << endl;
    
    s.Run();
    
    //buffer should now have 4 words 
    std::cout << "output buffer size: " << dmni.GetOutputBuffer()->size() << std::endl;
    
    //print buffer content
    uint32_t size = dmni.GetOutputBuffer()->size();
    for(uint32_t i = 0; i < size; i++){
        std::cout << dmni.GetOutputBuffer()->top() << " ";
        dmni.GetOutputBuffer()->pop();
    }
    std::cout << std::endl;
    
    //-------- dmni -> mem -------------------------------
    _addr = 0x100;
    //_op = OP_RECV;
    
    uint32_t res;
    
    dmni.GetInputBuffer()->push(1);
    dmni.GetInputBuffer()->push(2);
    dmni.GetInputBuffer()->push(3);
    dmni.GetInputBuffer()->push(4);

    s.Reset();
    //s.Run();
    
    mem.Read(_addr, (int8_t*)&res, 4);
    
    int8_t* ptr = (int8_t*)&res;
    std::cout << ptr[0] << " " << ptr[1] << " " << ptr[2] << " " << ptr[3] << std::endl;
    //----------------------------------------------------
    
	cout << "\nSimulation ended" << endl;
}
