#ifndef __SIMULATOR_H
#define __SIMULATOR_H

//lib dependencies
#include <iostream>
#include <array>
#include <queue>
#include <string>

//own api dependencies
#include <Event.h>

class Simulator{

private:

	/** queue that stores all events */
	std::priority_queue<Event> _queue;

	/** current simulated time */
	unsigned long long _globalTime;

	/** max time the simulation can reach */
	unsigned long long _timeout;
		
	/** execute event at top of event queue */
	void executeNext();

public:

	Simulator();

	unsigned long long Run(unsigned long long time = 100000);
    
        void Reset();
        unsigned long long GetGlobalTime();

	/** schedules a event */
	void Schedule(const Event& e);

	/** Dtor. */
	~Simulator();
};


#endif /* __SIMULATION_H */
