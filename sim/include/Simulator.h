#ifndef __SIMULATION_H
#define __SIMULATION_H

#include "Event.h"

#include <iostream>
#include <queue>
#include <string>

class Simulator{

    private:

        /** queue that stores all events */
	std::priority_queue<Event> queue;

	/** current simulated time */
	unsigned long long globalTime;

	/** max time the simulation can reach */
	unsigned long long timeout;
		
	/** execute event at top of event queue */
	void executeNext();

    public:
	Simulator(unsigned long long timeout);

	/** starts simulation */
	void Run();

	/** schedules a event */
	void Schedule(const Event& e);

	~Simulator();
};


#endif /* __SIMULATION_H */
