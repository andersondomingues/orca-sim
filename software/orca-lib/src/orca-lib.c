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
//TODO: generate automatically based on the IMPORT_APP("app_name") primitive

//#include "../../applications/example-counters/example-counters.h"
//#include "../../applications/narwal-launcher/narwal-launcher.h"
//#include "../../applications/producer-consumer/producer-consumer.h"
#include "../../applications/example-echo-print/example-echo-print.h"

//Task mapping routine and entry-point. Please note that 
//task mapping is done through software and the code below
//runs at the startup of each node. We select the tasks to 
//be loaded into each node according to nodes' ID. Startup
//routines that affect all applications can be handled here.
void app_main(void)
{
    //use hf_cpuid() to discrimate nodes

    hf_spawn(example_echo_print, 0, 0, 0, "echo-print", 4096);
}

