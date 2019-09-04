#ifndef _MORM_SP_H
#define _MORM_SP_H

#include <hellfire.h>
#include <noc.h>

#include "orca-lib.h"
#include "orca-hardware-counters.h"

#ifndef HRISCV_ENABLE_COUNTERS
#error ERROR_REQUIRE_HFRISC_COUNTERS
#endif

#ifndef ROUTER_ENABLE_COUNTERS
#error ERROR_REQUIRE_ROUTER_COUNTERS
#endif

#ifndef MEMORY_ENABLE_COUNTERS
#error ERROR_REQUIRE_MEMORY_COUNTERS
#endif

//morm sp routine
void morm_sp_task(void)	__attribute__((section (".tasks")));

#endif /* _MORM_SP_H */
