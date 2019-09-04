#include <Simulator.h>
#include <future>
#include <thread>
#include <chrono>
#include <functional>

using namespace std;

//check whether signal commit is necessary		
//if(_globalTime 	
//_globalTime = e.time;

/**
 * Defaul constructor
 */
Simulator::Simulator(){
    _globalTime = 0; 
	_epochs = 0;
	_timeout = 1;
}

SimulationTime Simulator::Run(SimulationTime time){
	
	_globalTime = 0;
	_timeout = time;

	std::future<SimulationTime> et;

	while(_globalTime <= _timeout){
	
		#ifdef URSA_QUEUE_SIZE_CHECKING
		if(_queue.size() <= 0)
			break;
		#endif

		//get next event to be processed
		Event e = _queue.top();

		//check whether a cycle has ended and 
		//commit signals
		//if(_globalTime < e.time){
			_globalTime = e.time;
			//this->CommitSignals();
		//}

		//execute delta using future
		//et = std::async(
		//	std::launch::async, // | std::launch::deferred,
		//	[e](){ return e.timedModel->Run(); }
		//);
		
		//remove old event
		_queue.pop();		
		
		//push new one 	
		//et.wait();
		
		//e.time += et.get();
		e.time += e.timedModel->Run();
		_queue.push(e);
	}
	
	return _globalTime;
}

//void Simulators::_commit_signals(){
//	return;	
//}


SimulationTime Simulator::NextEpoch(){

	//commit all signals
	//_commit_signal();

	//get the number of elements scheduled
	uint32_t queue_size = _queue.size();

	//time discount
	SimulationTime discount = _globalTime;//-1;
	
	//std::cout << "discount: " << discount << std::endl;
	
	//create a new queue and reschedule events
	Event tmp_queue[queue_size];
	
	//store events in an array so that we can update
	//them without messing up with the priority queue	
	for(uint32_t i = 0; i < queue_size; i++){
		tmp_queue[i] = _queue.top();

		//std::cout << "time = " << tmp_queue[i].time << std::endl;
		tmp_queue[i].time -= discount;

		_queue.pop();
	}
	
	//put update events back in simulator's queue
	for(uint32_t i = 0; i < queue_size; i++)
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
 * @param Event to run.*/
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
