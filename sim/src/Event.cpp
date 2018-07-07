#include "Event.h"

bool Event::operator<(const Event& e) const{
	return (this->time > e.time);
}

Event::Event(long long time, Process* p){
	this->time = time;
	this->process = p;
}

