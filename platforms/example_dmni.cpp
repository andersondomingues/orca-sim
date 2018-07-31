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

#include <DmniModel.h>

#define CYCLES_TO_SIM 100000
#define MEM_SIZE  0x00100000


//this program tests the DmniModel class 
int main(int argc, char** argv){

	cout << "Simulation stated" << endl;
	
	//creates a new simulator
	Simulator s = Simulator(CYCLES_TO_SIM);


    MemoryModel mem = MemoryModel("mem1", MEM_SIZE, true);
    s.Schedule(Event(0, &mem));


    bool _intr;
    uint32_t _addr, _length, _op;
    
    DmniModel dmni = DmniModel("dmni1");
    dmni.SetIntr(&_intr);
    dmni.SetAddress(&_addr);
    dmni.SetInputBuffer(new Buffer("dmnibuf"));
    dmni.SetLength(&_length);
    dmni.SetMemoryModel(&mem);
    dmni.SetOperation(&_op);
    dmni.Reset();
    
    _addr = 0x0;
    _length = 4; //4 bytes
    _op = OP_SEND;
    
    s.Schedule(Event(0, &dmni));
        
    //--------- mem -> dmni ------------------------------
    uint32_t data = 0x0F; 
    mem.Write(0x0, (int8_t*)&data, 4); //write 4 bytes to 0x0, 0x1, 0x2 and 0x3
    
    //print dmni state
    std::cout << "output buffer size: " << dmni.GetOutputBuffer()->size() << std::endl;
    
    //dmni.SetConfigure(0x0, 4); //read 4 bytes, from 0x0 to 0x3
    //dmni.SetWrite(true); //write to the output
    
    s.Run();
    
    //print buffers
    std::cout << "output buffer size: " << dmni.GetOutputBuffer()->size() << std::endl;
    for(uint32_t i = 0; i < dmni.GetOutputBuffer()->size(); i++){
        std::cout << dmni.GetOutputBuffer()->top() << std::endl;
        dmni.GetInputBuffer()->pop();
    }
    
    //-------- dmni -> mem -------------------------------
    
    //----------------------------------------------------
    
	cout << "\nSimulation ended" << endl;
}
