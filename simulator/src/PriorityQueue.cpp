#include <PriorityQueue.h>

/**
 * Default ctor.
 */
PriorityQueue::PriorityQueue(){
    _first = nullptr;
    _last  = nullptr;
    _count = 0;
}

/**
 * @param e Event to be pushed to the queue
 */
void PriorityQueue::Push(Event* e){

    switch(_count){
        
        case 0:
            PriorityQueueNode* n = new PriorityQueueNode(e);
            _first = n;
            _last = n;
            break;

        case 1:
            PriorityQueueNode* n = new PriorityQueueNode(e);
            if(e.time > _first->event->time){
                
            }else{


            }
    



    }

    if(_count == 0){
           }else if{
        


    }


}
