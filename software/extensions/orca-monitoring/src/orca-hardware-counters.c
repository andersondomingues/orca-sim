#include "orca-core.h"
#include "orca-hardware-counters.h"

#ifdef MEMORY_ENABLE_COUNTERS
volatile uint32_t* M0_COUNTER_STORE = (volatile uint32_t*) M0_COUNTER_STORE_ADDR;
volatile uint32_t* M0_COUNTER_LOAD  = (volatile uint32_t*) M0_COUNTER_LOAD_ADDR;
volatile uint32_t* M1_COUNTER_STORE = (volatile uint32_t*) M1_COUNTER_STORE_ADDR;
volatile uint32_t* M1_COUNTER_LOAD  = (volatile uint32_t*) M1_COUNTER_LOAD_ADDR;
volatile uint32_t* M2_COUNTER_STORE = (volatile uint32_t*) M2_COUNTER_STORE_ADDR;
volatile uint32_t* M2_COUNTER_LOAD  = (volatile uint32_t*) M2_COUNTER_LOAD_ADDR;
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
#endif

#ifdef ROUTER_ENABLE_COUNTERS
volatile uint32_t* ROUTER_COUNTER_ACTIVE = (volatile uint32_t*) ROUTER_COUNTER_ACTIVE_ADDR;
#endif
