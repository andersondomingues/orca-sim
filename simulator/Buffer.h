#ifndef __BUFFER_H
#define __BUFFER_H

#include <iostream>
#include <queue>
#include <stdint.h>

class Buffer{

private:
        std::string _name;
        std::queue<uint32_t>* _queue;

    public:
        Buffer(std::string name = "");
        ~Buffer();

        uint32_t top();
        void pop();
        void push(uint32_t);
        uint32_t size();
};

#endif
