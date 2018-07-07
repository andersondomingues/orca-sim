#include "Simulator.h"

using namespace std;

Simulator::Simulator(unsigned long long  timeout){

	this->timeout = timeout;
	this->globalTime = 0;
}

/**
 * Simulates the scheduled events. Initial events must 
 * be scheduled in constructor */
void Simulator::Run(){

	cout << "Simulation started" << endl;

	while(this->queue.size() > 0 && this->globalTime < this->timeout){
		cout << this->globalTime << ": " << this->queue.top().process->name << endl;
		this->executeNext();
	}

	cout << "Simulation finished at " << this->globalTime << endl;
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

	//cout << this->globalTime << ": " << e.process->name << endl;

	//process it
	long long int interval = e.process->Run();
	this->queue.push(Event(this->globalTime + interval, e.process));

	//remove it from the top of the queue
	this->queue.pop();
}	

Simulator::~Simulator(){

}
