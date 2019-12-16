#include <Simulator.h>

using namespace std;

Simulator::Simulator(){
    _globalTime = 0;
	_timeout = 1;
	_epochs = 0;
}

SimulationTime Simulator::Run(SimulationTime time){
	
	_globalTime = 0;
	_timeout = time;

	#ifdef URSA_QUEUE_SIZE_CHECKING
	while(_queue.size() > 0 && _globalTime <= _timeout){
	#else
	while(_globalTime <= _timeout){
	#endif

		//get next event to be processed
		Event e = _queue.top();
	
		_globalTime = e.time;
		e.time += e.timedModel->Run();
	
		//remove old event
		_queue.pop();		
		
		//push new one 
		_queue.push(e);
	}
	
	return _globalTime;
}

SimulationTime Simulator::NextEpoch(){

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
