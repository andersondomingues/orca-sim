#ifndef _MORM_SP_H
#define _MORM_SP_H

#include <hellfire.h>
#include <noc.h>

#define MEM0_COUNTERS_STORE_ADDR (uint32_t*) 0x81000000
#define MEM0_COUNTERS_LOAD_ADDR  (uint32_t*) 0x81000004
#define MEM1_COUNTERS_STORE_ADDR (uint32_t*) 0x81000008
#define MEM1_COUNTERS_LOAD_ADDR  (uint32_t*) 0x8100000C
#define MEM2_COUNTERS_STORE_ADDR (uint32_t*) 0x81000010
#define MEM2_COUNTERS_LOAD_ADDR  (uint32_t*) 0x81000014

#define CPU_COUNTERS_IARITH_ADDR     (uint32_t*) 0x81000100 
#define CPU_COUNTERS_ILOGICAL_ADDR   (uint32_t*) 0x81000104
#define CPU_COUNTERS_ISHIFT_ADDR     (uint32_t*) 0x81000108
#define CPU_COUNTERS_IBRANCHES_ADDR  (uint32_t*) 0x8100010C
#define CPU_COUNTERS_IJUMPS_ADDR     (uint32_t*) 0x81000110
#define CPU_COUNTERS_ILOADSTORE_ADDR (uint32_t*) 0x81000114

#define ROUTER_COUNTERS_ACTIVE_ADDR (uint32_t*) 0x81000200

//morm sp routine
void morm_sp_task(void)	__attribute__((section (".tasks")));

#endif /* _MORM_SP_H */