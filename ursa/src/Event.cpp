#include "Event.h"

bool Event::operator<(const Event& e) const{
	return (this->time > e.time);
}

Event::Event(SimulationTime t, TimedModel* p){
	this->time = t;
	this->timedModel = p;
}

//ctor required for arrays, intentionally left blank
Event::Event(){
	this->time = 0;
	this->timedModel = nullptr;
}
