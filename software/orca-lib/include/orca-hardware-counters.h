#ifndef __ORCA_HARDWARE_COUNTERS_H
#define __ORCA_HARDWARE_COUNTERS_H

//counters for memory modules
#define M0_COUNTER_STORE (volatile uint32_t*) 0x81000000
#define M0_COUNTER_LOAD  (volatile uint32_t*) 0x81000004
#define M1_COUNTER_STORE (volatile uint32_t*) 0x81000008
#define M1_COUNTER_LOAD  (volatile uint32_t*) 0x8100000C
#define M2_COUNTER_STORE (volatile uint32_t*) 0x81000010
#define M2_COUNTER_LOAD  (volatile uint32_t*) 0x81000014

//counters for the cpu
#define CPU_COUNTER_ARITH     (volatile uint32_t*) 0x81000100 
#define CPU_COUNTER_LOGICAL   (volatile uint32_t*) 0x81000104
#define CPU_COUNTER_SHIFT     (volatile uint32_t*) 0x81000108
#define CPU_COUNTER_BRANCHES  (volatile uint32_t*) 0x8100010C
#define CPU_COUNTER_JUMPS     (volatile uint32_t*) 0x81000110
#define CPU_COUNTER_LOADSTORE (volatile uint32_t*) 0x81000114

//counters for the router
#define ROUTER_COUNTER_ACTIVE (volatile uint32_t*) 0x81000200

//macros for reading and writing counters
#define GetHwCounter(x) *x
#define SetHwCounter(x, y) (*x = y)

#endif /* __ORCA_HARDWARE_COUNTERS_H */