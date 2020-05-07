#ifndef __WORDSIZE
#define __WORDSIZE 32
#warning "__WORDSIZE is undefined. Defaulting simtime_t to 64-bit word (uint64_t)."
#endif

#if __WORDSIZE == 32
typedef uint32_t SimulationTime;
#elif __WORDSIZE == 64
typedef uint64_t SimulationTime;
#else
#warning  __WORDSIZE is undefined. Defaulting simtime_t to 64-bit word (uint64_t).
typedef uint64_t SimulationTime;
#endif
