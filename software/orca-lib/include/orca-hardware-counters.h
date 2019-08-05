#ifndef __ORCA_HARDWARE_COUNTERS_H
#define __ORCA_HARDWARE_COUNTERS_H

//counters for memory modules
extern volatile uint32_t* M0_COUNTER_STORE;
extern volatile uint32_t* M0_COUNTER_LOAD;
extern volatile uint32_t* M1_COUNTER_STORE;
extern volatile uint32_t* M1_COUNTER_LOAD;
extern volatile uint32_t* M2_COUNTER_STORE;
extern volatile uint32_t* M2_COUNTER_LOAD;

//counters for the cpu
extern volatile uint32_t* CPU_COUNTER_ARITH;
extern volatile uint32_t* CPU_COUNTER_LOGICAL;
extern volatile uint32_t* CPU_COUNTER_SHIFT;
extern volatile uint32_t* CPU_COUNTER_BRANCHES;
extern volatile uint32_t* CPU_COUNTER_JUMPS;
extern volatile uint32_t* CPU_COUNTER_LOADSTORE;

extern volatile uint32_t* ROUTER_COUNTER_ACTIVE;

//macros for reading and writing counters
#define GetHwCounter(x) *x
#define SetHwCounter(x, y) (*x = y)

#endif /* __ORCA_HARDWARE_COUNTERS_H */
