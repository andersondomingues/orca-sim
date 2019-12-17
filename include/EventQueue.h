#ifndef _PRIORITY_QUEUE_H
#define _PRIORITY_QUEUE_H

#include <Event.h>

class EventQueue{

private:
	Event* _heap[URSA_MAX_QUEUE_SIZE];
    unsigned short int _count; 	

public:

	EventQueue();
	~EventQueue();

	#pragma region QUEUE_INTERFACE
	/**
	 * Pushes a new value to the queue.
	 * @param t element to be pushed to queue.
	 * @return TRUE if the element were sucefully 
	 * inserted, otherwise FALSE. 
	 * */
	bool Push(Event* t);

	/**
	 * Pops an event from the queue.
	 * @return The event with least time tag.
	 * */
	Event* Pop();
	#pragma endregion

	#pragma region HEAP_IMPL_METHODS
	void _Heapify(unsigned short int index);

	#pragma endregion

};

#endif