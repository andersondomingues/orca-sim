#ifndef __ORCA_HARDWARE_COUNTERS_H
#define __ORCA_HARDWARE_COUNTERS_H

#define M0_COUNTER_STORE_ADDR (0x403F0010)
#define M0_COUNTER_LOAD_ADDR  (0x403F0014)
#define M1_COUNTER_STORE_ADDR (0x403F0018)
#define M1_COUNTER_LOAD_ADDR  (0x403F001C)
#define M2_COUNTER_STORE_ADDR (0x403F0020)
#define M2_COUNTER_LOAD_ADDR  (0x403F0024)

#define CPU_COUNTER_ARITH_ADDR     (0x403F0028)
#define CPU_COUNTER_LOGICAL_ADDR   (0x403F002C)
#define CPU_COUNTER_SHIFT_ADDR     (0x403F0030)
#define CPU_COUNTER_BRANCHES_ADDR  (0x403F0034)
#define CPU_COUNTER_JUMPS_ADDR     (0x403F0038)
#define CPU_COUNTER_LOADSTORE_ADDR (0x403F003C)

#define ROUTER_COUNTER_ACTIVE_ADDR (0x403F0040)

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
