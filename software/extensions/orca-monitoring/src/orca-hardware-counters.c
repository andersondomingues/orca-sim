#include "orca-core.h"
#include "orca-hardware-counters.h"

#ifdef MEMORY_ENABLE_COUNTERS
volatile uint32_t* M0_COUNTER_STORE = (volatile uint32_t*) M0_COUNTER_STORE_ADDR;
volatile uint32_t* M0_COUNTER_LOAD  = (volatile uint32_t*) M0_COUNTER_LOAD_ADDR;
volatile uint32_t* M1_COUNTER_STORE = (volatile uint32_t*) M1_COUNTER_STORE_ADDR;
volatile uint32_t* M1_COUNTER_LOAD  = (volatile uint32_t*) M1_COUNTER_LOAD_ADDR;
volatile uint32_t* M2_COUNTER_STORE = (volatile uint32_t*) M2_COUNTER_STORE_ADDR;
volatile uint32_t* M2_COUNTER_LOAD  = (volatile uint32_t*) M2_COUNTER_LOAD_ADDR;

uint32_t GetCounter_M0_Stores(){ return *M0_COUNTER_STORE; }
uint32_t GetCounter_M0_Loads(){  return *M0_COUNTER_LOAD; }
uint32_t GetCounter_M1_Stores(){ return *M1_COUNTER_STORE; }
uint32_t GetCounter_M1_Loads(){  return *M1_COUNTER_LOAD; }
uint32_t GetCounter_M2_Stores(){ return *M2_COUNTER_STORE; }
uint32_t GetCounter_M2_Loads(){  return *M2_COUNTER_LOAD; }

#endif

#ifdef HFRISCV_ENABLE_COUNTERS
volatile uint32_t* CPU_COUNTER_ARITH     = (volatile uint32_t*) CPU_COUNTER_ARITH_ADDR;
volatile uint32_t* CPU_COUNTER_LOGICAL   = (volatile uint32_t*) CPU_COUNTER_LOGICAL_ADDR;
volatile uint32_t* CPU_COUNTER_SHIFT     = (volatile uint32_t*) CPU_COUNTER_SHIFT_ADDR;
volatile uint32_t* CPU_COUNTER_BRANCHES  = (volatile uint32_t*) CPU_COUNTER_BRANCHES_ADDR;
volatile uint32_t* CPU_COUNTER_JUMPS     = (volatile uint32_t*) CPU_COUNTER_JUMPS_ADDR;
volatile uint32_t* CPU_COUNTER_LOADSTORE = (volatile uint32_t*) CPU_COUNTER_LOADSTORE_ADDR;
volatile uint32_t* CPU_COUNTER_HOSTTIME  = (volatile uint32_t*) CPU_COUNTER_HOSTTIME_ADDR;
volatile uint32_t* CPU_COUNTER_CYCLES_TOTAL = (volatile uint32_t*) CPU_COUNTER_CYCLES_TOTAL_ADDR;
volatile uint32_t* CPU_COUNTER_CYCLES_STALL = (volatile uint32_t*) CPU_COUNTER_CYCLES_STALL_ADDR;

uint32_t GetCounter_CPU_ArithInstr(){  return *CPU_COUNTER_ARITH; }
uint32_t GetCounter_CPU_LogicInstr(){  return *CPU_COUNTER_LOGICAL; }
uint32_t GetCounter_CPU_ShiftInstr(){  return *CPU_COUNTER_SHIFT; }
uint32_t GetCounter_CPU_BranchInstr(){ return *CPU_COUNTER_BRANCHES; }
uint32_t GetCounter_CPU_JumpsInstr(){  return *CPU_COUNTER_JUMPS; }
uint32_t GetCounter_CPU_LoadStoreInstr(){ return *CPU_COUNTER_LOADSTORE; }
inline uint32_t GetCounter_CPU_HostTime(){ return *CPU_COUNTER_HOSTTIME; }
uint32_t GetCounter_CPU_CyclesTotal(){ return *CPU_COUNTER_CYCLES_TOTAL; }
uint32_t GetCounter_CPU_CyclesStall(){ return *CPU_COUNTER_CYCLES_STALL; }

#endif

#ifdef ROUTER_ENABLE_COUNTERS
volatile uint32_t* ROUTER_COUNTER_ACTIVE = (volatile uint32_t*) ROUTER_COUNTER_ACTIVE_ADDR;
uint32_t GetCounter_ROUTER_Active(){ return *ROUTER_COUNTER_ACTIVE; }
#endif

//funcs