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
 
#ifndef _EXAMPLE_COUNTERS_H
#define _EXAMPLE_COUNTERS_H

#include <hellfire.h>
#include <noc.h>

#include "orca-lib.h"
#include "orca-hardware-counters.h"

#ifndef HFRISC_ENABLE_COUNTERS
#error "This application requires HFRISC_ENABLE_COUNTERS to be set."
#endif

#ifndef MEMORY_ENABLE_COUNTERS
#error "This application requires MEMORY_ENABLE_COUNTERS to be set."
#endif

#ifndef ROUTER_ENABLE_COUNTERS
#error "This application requires ROUTER_ENABLE_COUNTERS to be set."
#endif

void counters_report(void) __attribute__((section (".tasks")));

#endif /* _MORM_SP_H */
