#ifndef __ORCA_LIB_H
#define __ORCA_LIB_H

#include "hellfire.h"

//!!! must be changed in noc.c as well
#define COMM_ID         (0x403F0010) 
#define COMM_HOSTTIME   (0x403F0014)

//apps' entry point
void app_main(void);

#endif /* __ORCA_LIB_H */
