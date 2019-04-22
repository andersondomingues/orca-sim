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

	#ifndef URSA_SKIP_EMPTY_QUEUE_CHECKING
	while(_globalTime <= _timeout){
	#else
	while(_queue.size() > 0 && _globalTime <= _timeout){
	#endif

		//get next event to be processed
		Event e = _queue.top();
	
		_globalTime = e.time;

		_queue.push( //push new event to queue
			Event(	 //event is scheduled to X cycles ahead of current time
				_globalTime + e.timedModel->Run(), 
				e.timedModel
			)
		);
		
		_queue.pop();
	}
	
	return _globalTime;
}

/**
 * @brief Schedule an event to run in a certain period of time
 * @param Event to run.*/
void Simulator::Schedule(const Event& e){

	#ifndef OPT_SKIP_ZERO_TIME_CHECKING
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
 * @brief Dtor. */
Simulator::~Simulator(){

}
