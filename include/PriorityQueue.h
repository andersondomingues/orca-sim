#ifndef __PRIORITY_QUEUE_H
#define __PRIORITY_QUEUE_H


/**
 * This class models a priority queue node,
 * holding both an event and its execution
 * time. The execution time is the sorting
 * criterion.
 */
class PriorityQueueNode{

public:
    /**
     * Time in which the event must be executed.
     */
    SimulationTime time;
    
    /**
     * Event that will be be executed.
     */
    Event* event;

    /**
     * Pointer to the antecessor node in the queue. Must be 
     * equals to <nullptr> if none.
     */
    PriorityQueue* previous;

    /**
     * Pointer to the successor node in the queue. Must be
     * equals to <nullptr> if none.
     */
    PriorityQueue* next;
};

/**
 * This class models a priority queue of events, in which the 
 * sorting criterion is the execution time of events.
 */
class PriorityQueue{

private:
    /**
     * First node of the queue (top). Must be <nullptr> if none.
     */
    PriorityQueueNode* _first;

    /**
     * Last node of the queue (bottom). Must be <nullptr> if none.
     */
    PriorityQueueNode* _last;

    /**
     * Number of elements in the queue.
     */
    uint32_t _count;

public:
    /**
     * Returns the element from the top of the queue. The element is NOT 
     * removed!
     * @returns The element from the top of the queue, or nullptr if none.
     */
    Event* Top();

    /**
     * Removes an element from top of the queue if the queue is not empty.
     * If empty, an exception should be triggered.
     */
    void Pop();

    /**
     * Push an element to the queue. The element is placed in an arbitrary
     * position according to the sorting criterion.
     * @p Event to be added to the queue.
     */
    void Push(Event* p);
};


#endif /* __PRIORITY_QUEUE_H */
