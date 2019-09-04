/*
 * Implementation file for drone-pid task.
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
 

#include "drone-pid.h"

#include <hellfire.h>
#include <noc.h>

void dronepid(){

	dummy t();

    int8_t buf[500];
    uint16_t cpu, port, size;
    int16_t val;

    if(hf_comm_create(hf_selfid(), 2000, 0))
        panic(0xff);

    while(1){

        int32_t i = hf_recvprobe();

        //check whether a packet arrived 
        if(i >= 0){

            val = hf_recv(&cpu, &port, buf, &size, i);

            if(val){
                printf("hf_recv(): error %d\n", val);

            }else{

                drone_pid_data_t data;
                data = *(drone_pid_data_t*)buf;

				printf("drone-pid: %d\n", data.d);

            }
        }
    }
}
