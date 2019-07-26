/*
 * Implementation file for ORCA-LIB library.
 * Copyright (C) 2018-2019 Anderson Domingues, <ti.andersondomingues@gmail.com>
 * This file is part of project URSA (http://https://github.com/andersondomingues/ursa).
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. */
 
//basic resources
#include "orca-lib.h"

//extended functionalities
#include "orca-hardware-counters.h"

//application-specific header
#include "../../applications/example-systime/example-systime.h"

//Task mapping routine and entry-point. Please note that 
//task mapping is done through software and the code below
//runs at the startup of each node. We select the tasks to 
//be loaded into each node according to nodes' ID. Startup
//routines that affect all applications can be handled here.
void app_main(void)
{
	hf_spawn(report_systime, 100, 99, 99, "report systime", 4096);

	 //use hf_cpuid() to discrimate nodes
	 switch(hf_cpuid()){
	 case 1: 
	 	 //hf_spawn(dronespammer, 0, 0, 0, "drone spammer", 4096);
	 	 break;
	 case 3: 	 
		 //hf_spawn(droneekf, 0, 0, 0, "drone ekf", 4096);
		 break;
	 case 2:
	 	 //hf_spawn(dronepid, 0, 0, 0, "drone pid", 4096);
         //hf_spawn(example_echo_print, 0, 0, 0, "echo-print", 4096);
	 	 break;
	 default:
		 break;
	 }

	 //allocating real-time for ~90% (9/10)
	 //10 : period
	 // 9 : capacity
	 //10 : dealine

	 //to allocate best-effort tasks, use hf_spawn(tskname, 0, 0, 0, "name", stacksize);	 
}
	 
	 

