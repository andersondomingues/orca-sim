#include <Event.h>

bool Event::operator<(const Event& e) const{
	return (this->time > e.time);
}

Event::Event(SimulationTime time, TimedModel* p){
	this->time = time;
	this->timedModel = p;
}

Event::Event(){}