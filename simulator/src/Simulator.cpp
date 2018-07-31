#include "Simulator.h"

using namespace std;

Simulator::Simulator(unsigned long long  timeout){

	this->timeout = timeout;
	Reset();
}

void Simulator::Reset(){
    this->globalTime = 0;
}

/**
 * Simulates the scheduled events. Initial events must 
 * be scheduled in constructor */
void Simulator::Run(){

	while(this->queue.size() > 0 && this->globalTime < this->timeout){
		//cout << this->globalTime << ": " << this->queue.top().process->name << endl;
		this->executeNext();
	}
}

/**
 * Schedules an event */
void Simulator::Schedule(const Event& e){
	this->queue.push(e);
}

/**
 * Removes the event on top of the queue and executes it */
void Simulator::executeNext(){
	
	//get next event to be processed
	Event e = this->queue.top();
	this->globalTime = e.time;

	//process it
	long long int interval = e.process->Run();
	this->queue.push(Event(this->globalTime + interval, e.process));

	//remove it from the top of the queue
	this->queue.pop();
}

Simulator::~Simulator(){

}
