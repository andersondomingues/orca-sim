#include "Buffer.h"


Buffer::Buffer(){

}

Buffer::~Buffer(){

}

void Buffer::pop(){
    this->queue.pop();
}

void Buffer::push(uint32_t e){
    this->queue.push(e);
}

uint32_t Buffer::top(){
    return this->queue.front();
}

uint32_t Buffer::size(){
    return this->queue.size();
}
