#ifndef __ORCA_HARDWARE_COUNTERS_H
#define __ORCA_HARDWARE_COUNTERS_H

//0x403F1xxx => memory mapped counters
#ifdef MEMORY_ENABLE_COUNTERS
#define M0_COUNTER_STORE_ADDR (0x40411010)
#define M0_COUNTER_LOAD_ADDR  (0x40411014)
#define M1_COUNTER_STORE_ADDR (0x40411018)
#define M1_COUNTER_LOAD_ADDR  (0x4041101C)
#define M2_COUNTER_STORE_ADDR (0x40411020)
#define M2_COUNTER_LOAD_ADDR  (0x40411024)

extern volatile uint32_t* M0_COUNTER_STORE;
extern volatile uint32_t* M0_COUNTER_LOAD;
extern volatile uint32_t* M1_COUNTER_STORE;
extern volatile uint32_t* M1_COUNTER_LOAD;
extern volatile uint32_t* M2_COUNTER_STORE;
extern volatile uint32_t* M2_COUNTER_LOAD;

uint32_t GetCounter_M0_Stores();
uint32_t GetCounter_M0_Loads();
uint32_t GetCounter_M1_Stores();
uint32_t GetCounter_M1_Loads();
uint32_t GetCounter_M2_Stores();
uint32_t GetCounter_M2_Loads();

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

extern volatile uint32_t* CPU_COUNTER_ARITH;
extern volatile uint32_t* CPU_COUNTER_LOGICAL;
extern volatile uint32_t* CPU_COUNTER_SHIFT;
extern volatile uint32_t* CPU_COUNTER_BRANCHES;
extern volatile uint32_t* CPU_COUNTER_JUMPS;
extern volatile uint32_t* CPU_COUNTER_LOADSTORE;
extern volatile uint32_t* CPU_COUNTER_HOSTTIME;
extern volatile uint32_t* CPU_COUNTER_CYCLES_TOTAL;
extern volatile uint32_t* CPU_COUNTER_CYCLES_STALL;

uint32_t GetCounter_CPU_ArithInstr();
uint32_t GetCounter_CPU_LogicInstr();
uint32_t GetCounter_CPU_ShiftInstr();
uint32_t GetCounter_CPU_BranchInstr();
uint32_t GetCounter_CPU_JumpsInstr();
uint32_t GetCounter_CPU_LoadStoreInstr();
uint32_t GetCounter_CPU_HostTime();
uint32_t GetCounter_CPU_CyclesTotal();
uint32_t GetCounter_CPU_CyclesStall();

#endif

//0x403F15xx => router wires
#ifdef ROUTER_ENABLE_COUNTERS
#define ROUTER_COUNTER_ACTIVE_ADDR (0x40411500)

extern volatile uint32_t* ROUTER_COUNTER_ACTIVE;

uint32_t GetCounter_ROUTER_Active();
#endif




#endif /* __ORCA_HARDWARE_COUNTERS_H */
