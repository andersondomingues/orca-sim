#ifndef _MORM_SP_H
#define _MORM_SP_H

#include <hellfire.h>
#include <noc.h>

#include "orca-lib.h"
#include "orca-hardware-counters.h"

//morm sp routine
void morm_sp_task(void)	__attribute__((section (".tasks")));

#endif /* _MORM_SP_H */
