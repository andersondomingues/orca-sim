/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software
 * http://https://github.com/andersondomingues/orca-mpsoc
 * http://https://github.com/andersondomingues/orca-tools
 *
 * Copyright (C) 2018-2020 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. 
******************************************************************************/
#ifndef PLATFORMS_SINGLE_CORE_NN_INCLUDE_MEMORYMAP_H_
#define PLATFORMS_SINGLE_CORE_NN_INCLUDE_MEMORYMAP_H_

#define SIGNAL_CPU_STALL    0x40412000  // 8 bits
#define SIGNAL_CPU_INTR     0x40412001
#define SIGNAL_DMA_PROG     0x40412002
// 0x4041003
#define DMA_BURST_SIZE      0x40412004  // 32 bits
#define DMA_NN_SIZE         0x40412008
#define DMA_OUT_SIZE        0x4041200C
#define DMA_MAC_OUT_ARRAY   0x40412010
// DMA_MAC_OUT_ARRAY0           0x40412010
// DMA_MAC_OUT_ARRAY1           0x40412014
// ...
// DMA_MAC_OUT_ARRAY31           0x40412010 + (0x20 * 4) -1

#define DMA_BURST_SIZE      0x40412004  // 32 bits
#define DMA_NN_SIZE         0x40412008
#define DMA_OUT_SIZE        0x4041200C
#define DMA_MAC_OUT_ARRAY   0x40412010
// DMA_MAC_OUT_ARRAY0           0x40412010
// DMA_MAC_OUT_ARRAY1           0x40412014
//...
// DMA_MAC_OUT_ARRAY31           0x40412010 + (0x20 * 4) -1

// 0x40411xxx => memory mapped counters
#ifdef MEMORY_ENABLE_COUNTERS
#define M0_COUNTER_STORE_ADDR (0x40411010)
#define M0_COUNTER_LOAD_ADDR  (0x40411014)
#define M1_COUNTER_STORE_ADDR (0x40411018)
#define M1_COUNTER_LOAD_ADDR  (0x4041101C)
#define M2_COUNTER_STORE_ADDR (0x40411020)
#define M2_COUNTER_LOAD_ADDR  (0x40411024)
#endif

#ifdef HFRISCV_ENABLE_COUNTERS
#define CPU_COUNTER_ARITH_ADDR     (0x40411128)
#define CPU_COUNTER_LOGICAL_ADDR   (0x4041112C)
#define CPU_COUNTER_SHIFT_ADDR     (0x40411130)
#define CPU_COUNTER_BRANCHES_ADDR  (0x40411134)
#define CPU_COUNTER_JUMPS_ADDR     (0x40411138)
#define CPU_COUNTER_LOADSTORE_ADDR (0x4041113C)
#define CPU_COUNTER_HOSTTIME_ADDR  (0x40411140)
#define CPU_COUNTER_CYCLES_TOTAL_ADDR (0x40411144)
#define CPU_COUNTER_CYCLES_STALL_ADDR (0x40411148)
#endif

// 0x403F15xx => router wires
#ifdef ROUTER_ENABLE_COUNTERS
#define ROUTER_COUNTER_ACTIVE_ADDR (0x40411500)
#endif

// NN memory configuration. TOTAL_NN_MEM_SIZE and SIMD_SIZE can be changed
// in design time
#define TOTAL_NN_MEM_SIZE 4 * 1024 * 1024
#define SIMD_SIZE 32
#define NN_MEM_SIZE_PER_CHANNEL (TOTAL_NN_MEM_SIZE / 2)  /  SIMD_SIZE
#define MEMW_BASE 0x40500000
#define MEMI_BASE MEMW_BASE + (TOTAL_NN_MEM_SIZE/2)

#endif  // PLATFORMS_SINGLE_CORE_NN_INCLUDE_MEMORYMAP_H_
