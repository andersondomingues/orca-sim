/*
 * Implementation file for EXAMPLE-COUNTERS application.
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
 
#include "counter-test.h"

//Task for printing values store by CPU counters. 
void counter_test(void){

    for(;;){

        printf("MEM0: writes=%u, reads=%u\n", *M0_COUNTER_STORE, *M0_COUNTER_LOAD);
		printf("MEM1: writes=%u, reads=%u\n", *M1_COUNTER_STORE, *M1_COUNTER_LOAD);
		printf("MEM2: writes=%u, reads=%u\n", *M2_COUNTER_STORE, *M2_COUNTER_LOAD);
		printf("---\n");
	
		printf("CPU: arith=%u, logical=%u\n",   *CPU_COUNTER_ARITH, *CPU_COUNTER_LOGICAL);
		printf("CPU: shift=%u, branches=%u\n",  *CPU_COUNTER_SHIFT, *CPU_COUNTER_BRANCHES);
		printf("CPU: jumps=%u, loadstore=%u\n", *CPU_COUNTER_JUMPS, *CPU_COUNTER_LOADSTORE);
		printf("CPU: cycles=%u, stalls=%u\n", *CPU_COUNTER_CYCLES_TOTAL, *CPU_COUNTER_CYCLES_STALL);	
		printf("CPU: hosttime=%u\n", *CPU_COUNTER_HOSTTIME);
		printf("---\n");
	
		printf("ROUTER: active=%u\n", *ROUTER_COUNTER_ACTIVE);
		printf("---\n");
	}

	//hf_kill(hf_selfid());
	//while(1);
}
