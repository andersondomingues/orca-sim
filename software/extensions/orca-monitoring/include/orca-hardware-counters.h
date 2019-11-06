#ifndef __ORCA_HARDWARE_COUNTERS_H
#define __ORCA_HARDWARE_COUNTERS_H

#define M0_COUNTER_STORE_ADDR (0x403F1010)
#define M0_COUNTER_LOAD_ADDR  (0x403F1014)
#define M1_COUNTER_STORE_ADDR (0x403F1018)
#define M1_COUNTER_LOAD_ADDR  (0x403F101C)
#define M2_COUNTER_STORE_ADDR (0x403F1020)
#define M2_COUNTER_LOAD_ADDR  (0x403F1024)

#define CPU_COUNTER_ARITH_ADDR     (0x403F1028)
#define CPU_COUNTER_LOGICAL_ADDR   (0x403F102C)
#define CPU_COUNTER_SHIFT_ADDR     (0x403F1030)
#define CPU_COUNTER_BRANCHES_ADDR  (0x403F1034)
#define CPU_COUNTER_JUMPS_ADDR     (0x403F1038)
#define CPU_COUNTER_LOADSTORE_ADDR (0x403F103C)
#define CPU_COUNTER_HOSTTIME_ADDR  (0x403F1040)

#define ROUTER_COUNTER_ACTIVE_ADDR (0x403F1044)

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
extern volatile uint32_t* CPU_COUNTER_HOSTTIME;

extern volatile uint32_t* ROUTER_COUNTER_ACTIVE;

#endif /* __ORCA_HARDWARE_COUNTERS_H */
