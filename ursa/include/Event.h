#ifndef _event_h
#define _event_h

#include "TimedModel.h"
#include "SimulationTime.h"

class Event{

	public:
		/**
		 * Point in time when the event will trigger
		 * */
		SimulationTime time;

		/**
		 * Model whose activation function will be called 
		 * once the event triggers.
		 */
		TimedModel* timedModel;

		Event(SimulationTime, TimedModel*);
		Event(); //necessary for arrays

		/**
		 * Comparing operator. An event occurs first in time if its time 
		 * is less than the compared event (required by the priority queue, see
		 * <std::priority_queue>).
		 */
		bool operator<(const Event& e) const;
};

#endif /* event_h */
