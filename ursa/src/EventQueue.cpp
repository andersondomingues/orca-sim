#include <EventQueue.h>


EventQueue::EventQueue(){
    _heap = new T[URSA_MAX_QUEUE_SIZE];
    _count = 0; 
}

bool EventQueue::Push(Event* t){

    #ifdef URSA_QUEUE_PUSH_CHECKING
    if(_count == URSA_MAX_QUEUE_SIZE)
        return false;
    #endif    
    
    //insert at the last position and 
    //increment the number of elements
    _heap[_count] = t;
    
    int curr_pos = _count;

    _count++;

    //whether the min-heap property have
    //been violated, fix it
    int parent = GetParent(t);
    while(curr_pos != 0 && _heap[t].time < _heap[GetParent(t)].time){
        
        Event swp = _heap[t];
        _heap[t] = _heap[parent];
        _heap[parent] = swp;
        
        curr_pos = GetParent(t);
    }
}

Event* EventQueue::Pop(){

    #ifdef URSA_QUEUE_POP_CHECKING_EMPTY
    if(_count == 0)
        return nullptr;
    #endif

    #ifdef URSA_QUEUE_POP_CHECKING_SINGLE
    if(_count == 1){
        _count--;
        return _heap[0];
    }
    #endif
     
    //store value from root node
    Event* root = _heap[0];

    //remove that value
    _heap[0] = _heap[--_count]; 
    this->_Heapify(0);

    return root;
}

void EventQueue::_Heapify(uint32_t index){

    uint32_t l, r;
    l = left(index);
    r = right(index);

    uint32_t x;

    

}
