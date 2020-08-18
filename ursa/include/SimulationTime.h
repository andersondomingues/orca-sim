#include <stdint.h>
#ifdef __WORDSIZE

#if __WORDSIZE == 64
typedef uint64_t SimulationTime;
#else
typedef uint32_t SimulationTime;
#endif

#else
typedef uint32_t SimulationTime;
#endif
