#ifndef __EVENT_H
#define __EVENT_H

#include "Process.h"

class Event{

	public:
		long long time;
		Process* process;
		Event(long long time, Process* b);
		bool operator<(const Event& e) const;

};

#endif /* EVENT_H */
