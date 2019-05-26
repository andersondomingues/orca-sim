#include "orca-lib.h"
#include "orca-hardware-counters.h"

volatile uint32_t* M0_COUNTER_STORE = (volatile uint32_t*) 0x81000000;
volatile uint32_t* M0_COUNTER_LOAD  = (volatile uint32_t*) 0x81000004;
volatile uint32_t* M1_COUNTER_STORE = (volatile uint32_t*) 0x81000008;
volatile uint32_t* M1_COUNTER_LOAD  = (volatile uint32_t*) 0x8100000C;
volatile uint32_t* M2_COUNTER_STORE = (volatile uint32_t*) 0x81000010;
volatile uint32_t* M2_COUNTER_LOAD  = (volatile uint32_t*) 0x81000014;

volatile uint32_t* CPU_COUNTER_ARITH     = (volatile uint32_t*) 0x81000100;
volatile uint32_t* CPU_COUNTER_LOGICAL   = (volatile uint32_t*) 0x81000104;
volatile uint32_t* CPU_COUNTER_SHIFT     = (volatile uint32_t*) 0x81000108;
volatile uint32_t* CPU_COUNTER_BRANCHES  = (volatile uint32_t*) 0x8100010C;
volatile uint32_t* CPU_COUNTER_JUMPS     = (volatile uint32_t*) 0x81000110;
volatile uint32_t* CPU_COUNTER_LOADSTORE = (volatile uint32_t*) 0x81000114;

volatile uint32_t* ROUTER_COUNTER_ACTIVE = (volatile uint32_t*) 0x81000200;


