#ifndef _MEMORY_MAP_H
#define _MEMORY_MAP_H

#define MEM_BASE ORCA_MEMORY_BASE
#define MEM_SIZE ORCA_MEMORY_SIZE

//@TODO maybe move these to the Configuration.mk file
#define ORCA_MEMORY_BASE_1 0x00000000
#define ORCA_MEMORY_SIZE_1 0x00000080
#define ORCA_MEMORY_BASE_2 0x00000080
#define ORCA_MEMORY_SIZE_2 0x00000080

//control wires (cpu <-> dma)
#define SIGNAL_CPU_STALL    0x40410000  /* 8 bits */
#define SIGNAL_CPU_INTR     0x40410001
#define SIGNAL_SEND_STATUS  0x40410002
//0x40410003
#define SIGNAL_RECV_STATUS  0x40410004
//0x40410005
//0x40410006
//0x40410007
#define SIGNAL_PROG_SEND    0x40410008
#define SIGNAL_PROG_RECV    0x40410009
//0x4041000A
//0x4041000B
#define SIGNAL_PROG_ADDR    0x4041000C  /* 32 bits */
#define SIGNAL_PROG_SIZE    0x40410010

//tile indentifier, hardware fixed
#define MAGIC_TILE_ID       0x40411000

//0x403F1xxx => memory mapped counters
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

#endif
