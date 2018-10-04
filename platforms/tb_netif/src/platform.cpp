/*-------------------------------------------------------------------------
# This file is part of Project URSA. More information on the project
# can be found at URSA's repository at GitHub.
# 
# http://https://github.com/andersondomingues/ursa
# 
# Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#-------------------------------------------------------------------------*/

#include <cstdlib>
#include <TNetif.h>

typedef uint16_t FlitType;

int main(int argc, char** argv){

	//module to be tested
	TNetif* netif = new TNetif();
	
	//input buffer of the router
	Buffer<FlitType>* ob = new Buffer<FlitType>();
	
	//bind buffer to the dmni
	netif->SetOutputBuffer(ob);
	
	//test sending by generating 4 packets into input inputbuffer that 
	//should ba attached the the dma: [0x01][0x02][0x03][0x04]
	for(int i = 0; i < 4; i++)
		netif->GetInputBuffer()->push(i);
		
	//configure ni to send
	netif->SetIntrSend(new Comm<uint8_t>("intr_send", 0));
	netif->GetIntrSend()->Write(1);
	
	//simulate for 1000 cycles
	Simulator* s = new Simulator();
	s->Schedule(Event(1, netif));
	s->Run(1000);
	
	//print output buffer
		
}
