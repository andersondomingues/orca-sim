#ifndef _simulator_h
#define _simulator_h

//lib dependencies
#include <iostream>
#include <array>
#include <queue>
#include <string>

//own api dependencies
#include "Event.h"
#include "SimulationTime.h"

class Simulator{

private:

	SimulationTime _epochs;

	/** queue that stores all events */
	std::priority_queue<Event> _queue;

	/** current simulated time */
	SimulationTime _globalTime;

	/** max time the simulation can reach */
	SimulationTime _timeout;
		
	/** execute event at top of event queue */
	void executeNext();

public:
	/* ctor. */
	Simulator();

	/* run the simulation for <time> cycles. */
	SimulationTime Run(SimulationTime time = 100000);
    
	/* return current global time */
    SimulationTime GetGlobalTime();
    
   	/* return total epochs */
    SimulationTime GetEpochs();
    
    /* reset time, increment epochs and realign events */
    SimulationTime NextEpoch();

	/** schedules a event */
	void Schedule(const Event& e);

	/** Dtor. */
	~Simulator();
};


#endif /* __SIMULATION_H */
