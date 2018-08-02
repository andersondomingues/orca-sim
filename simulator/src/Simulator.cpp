#include "Simulator.h"

using namespace std;

Simulator::Simulator(){
	Reset();
}

void Simulator::Reset(){
    _globalTime = 0;
	_timeout = 0;
}

/**
 * @brief Simulates scheduled events.
 * @param time Number of cycles to be simulate.
 * @return the current time by the end of simulation.
 */
unsigned long long Simulator::Run(unsigned long long time){
	
	_timeout = _globalTime + time;

	while(_queue.size() > 0 && _globalTime < _timeout)
		this->executeNext();
	
	return _globalTime;
}

/**
 * @brief Schedule an event to run in a certain period of time
 * @param Event to run.*/
void Simulator::Schedule(const Event& e){
	_queue.push(e);
}

/**
 * @brief Removes the event on top of the queue and executes it */
void Simulator::executeNext(){
	
	//get next event to be processed
	Event e = _queue.top();
	_globalTime = e.time;

	//process it
	long long int interv = e.process->Run();
	_queue.push(Event(_globalTime + interv, e.process));

	//remove it from the top of the queue
	_queue.pop();
}

/**
 * @brief Dtor. */
Simulator::~Simulator(){

}
