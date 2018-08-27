#include <Simulator.h>

using namespace std;

Simulator::Simulator(){
	Reset();
}

void Simulator::Reset(){
    _globalTime = 0;
	_timeout = 1;
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

	#ifndef NOGUARDS
	if(e.time == 0){
		throw std::runtime_error("Simulator: unable to schedule "
			+ e.timedModel->GetName() + " to run in the past. " +
			"Events must be scheduled to run with time > 0." 
		);
	}
	#endif
	_queue.push(e);
}

/**
 * @brief Removes the event on top of the queue and executes it */
void Simulator::executeNext(){
	
	//get next event to be processed
	Event e = _queue.top();
	_queue.pop();
	
	_globalTime = e.time;

	//process it
	long long int interv = e.timedModel->Run();
	_queue.push(Event(_globalTime + interv, e.timedModel));
}

/**
 * @brief Dtor. */
Simulator::~Simulator(){

}