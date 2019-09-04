#ifndef _ORCA_SYSTIME_H
#define _ORCA_SYSTIME_H

//counters for memory modules
extern volatile uint32_t* SYSTIME_ADDR;

uint32_t GetHostTime();

#endif