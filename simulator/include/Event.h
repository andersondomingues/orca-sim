#ifndef __EVENT_H
#define __EVENT_H

#include "TimedModel.h"

class Event{

	public:
		long long time;
		TimedModel* timedModel;
		Event(long long time, TimedModel* b);
		bool operator<(const Event& e) const;
};

#endif /* EVENT_H */
