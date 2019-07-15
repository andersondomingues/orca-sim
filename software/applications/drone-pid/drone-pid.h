#ifndef __DRONE_PID_H
#define __DRONE_PID_H

#include <drone.h> /* includes drone_ekf_data_t type */
#include <hellfire.h>

#ifdef CPP
extern "C" {
#endif

void dronepid(void);


#ifdef CPP
}

class dummy{
private:
	uint32_t x;
};

#endif

#endif