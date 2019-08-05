#include "orca-lib.h"
#include "orca-hardware-counters.h"

volatile uint32_t* M0_COUNTER_STORE = (volatile uint32_t*) M0_COUNTER_STORE_ADDR;
volatile uint32_t* M0_COUNTER_LOAD  = (volatile uint32_t*) M0_COUNTER_LOAD_ADDR;
volatile uint32_t* M1_COUNTER_STORE = (volatile uint32_t*) M1_COUNTER_STORE_ADDR;
volatile uint32_t* M1_COUNTER_LOAD  = (volatile uint32_t*) M1_COUNTER_LOAD_ADDR;
volatile uint32_t* M2_COUNTER_STORE = (volatile uint32_t*) M2_COUNTER_STORE_ADDR;
volatile uint32_t* M2_COUNTER_LOAD  = (volatile uint32_t*) M2_COUNTER_LOAD_ADDR;

volatile uint32_t* CPU_COUNTER_ARITH     = (volatile uint32_t*) CPU_COUNTER_ARITH_ADDR;
volatile uint32_t* CPU_COUNTER_LOGICAL   = (volatile uint32_t*) CPU_COUNTER_LOGICAL_ADDR;
volatile uint32_t* CPU_COUNTER_SHIFT     = (volatile uint32_t*) CPU_COUNTER_SHIFT_ADDR;
volatile uint32_t* CPU_COUNTER_BRANCHES  = (volatile uint32_t*) CPU_COUNTER_BRANCHES_ADDR;
volatile uint32_t* CPU_COUNTER_JUMPS     = (volatile uint32_t*) CPU_COUNTER_JUMPS_ADDR;
volatile uint32_t* CPU_COUNTER_LOADSTORE = (volatile uint32_t*) CPU_COUNTER_LOADSTORE_ADDR;

volatile uint32_t* ROUTER_COUNTER_ACTIVE = (volatile uint32_t*) ROUTER_COUNTER_ACTIVE_ADDR;
