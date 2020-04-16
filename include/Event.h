#ifndef __EVENT_H
#define __EVENT_H

#include <TimedModel.h>
#include <SimulationTime.h>

class Event{

	public:
		SimulationTime time;
		TimedModel* timedModel;
		Event(SimulationTime time, TimedModel* b);
		Event(); //necessary for arrays
		bool operator<(const Event& e) const;
};

#endif /* EVENT_H */
