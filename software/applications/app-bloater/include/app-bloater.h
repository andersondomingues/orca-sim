/*
 * Header file for EXAMPLE-COUNTERS application.
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
#ifndef _APP_BLOATER_H
#define _APP_BLOATER_H

#include <hellfire.h>
#include <noc.h>

/*
//struct to deserialize reeived data
struct task_info{
	uint32_t func_addr;
	uint32_t stack_size;
	uint16_t rt_period;
	uint16_t rt_capacity;
	uint16_t rt_deadline;
	uint16_t ___padding0; //force data to be aligned
	uint32_t name_len;
	char* name_data;		
};
*/

void app_bloater(void); // __attribute__((section (".tasks")));

#endif /* _APP_BLOATER_H */
