#include "Buffer.h"

Buffer::Buffer(std::string name){
    this->_name = name;
    this->_queue = new std::queue<uint32_t>();
}

Buffer::~Buffer(){
    delete(_queue);
}

void Buffer::pop(){
    _queue->pop();
}

void Buffer::push(uint32_t e){
    _queue->push(e);
}

uint32_t Buffer::top(){
    return _queue->front();
}

/**
 * @brief Gets the size of the internal queue, measured in number of elements.
 * @return The number of elements into the internal queue. */
uint32_t Buffer::size(){
    return _queue->size();
}
