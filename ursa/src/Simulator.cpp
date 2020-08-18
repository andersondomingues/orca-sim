#include <Simulator.h>
#include <future>
#include <thread>
#include <chrono>
#include <functional>

using namespace std;

/**
 * Defaul constructor
 */
Simulator::Simulator(){
    _globalTime = 0; 
	_epochs = 0;
	_timeout = 1;
}

/**
  * @brief Runs an epoch of simulation
  * @param time Number of cycles to run
  * @return the time in which the last event was executed. Should
  * roughly correspond to <time>. */
SimulationTime Simulator::Run(SimulationTime time){

	_globalTime = 0;
	_timeout = time;

	//std::future<SimulationTime> et;

	while(_globalTime <= _timeout){
	
		#ifdef URSA_QUEUE_SIZE_CHECKING
		if(_queue.size() <= 0)
			break;
		#endif

		//get next event to be processed. Since we use a priority
		//queue to store events, the event nealy in time will be 
		//popped first
		Event e = _queue.top();
		_queue.pop();
		
		//update global time 
		_globalTime = e.time;

		//schedule current event to be executed after a certain number 
		//cycles, defined in the correspondent Run method
		e.time += e.timedModel->Run();
		
		//push event back to the queue
		_queue.push(e);
	}

	//return point in time in which the last event was executed
	return _globalTime;
}

/**
  * @brief Generate the next epoch for a simulation session.
  * @return ? */
SimulationTime Simulator::NextEpoch(){

	//get the number of elements scheduled
	int queue_size = static_cast<int>(_queue.size());

	//time that amount of time 
	SimulationTime discount = _globalTime - 1;
	
	//create a new queue and reschedule events
	Event tmp_queue[queue_size];
	
	//store events in an array so that we can update
	//them without messing up with the priority queue	
	for(int i = 0; i < queue_size; i++){

		tmp_queue[i] = _queue.top();	
		tmp_queue[i].time -= discount;
		
		_queue.pop();
	}

	//put update events back in simulator's queue
	for(int i = 0; i < queue_size; i++)
		_queue.push(tmp_queue[i]);

	//update epochs counters
	_epochs++;

	return _globalTime;
}

SimulationTime Simulator::GetGlobalTime(){
    return _globalTime;
}


SimulationTime Simulator::GetEpochs(){
	return _epochs;
}

/**
 * @brief Schedule an event to run in a certain period of time
 * @param e Event to run.*/
void Simulator::Schedule(const Event& e){

	#ifdef URSA_ZERO_TIME_CHECKING
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
