#ifndef __BUFFER_H
#define __BUFFER_H

#include <iostream>
#include <queue>
#include <stdint.h>

class Buffer{

    private:
        std::queue<uint32_t> queue;

    public:

        Buffer();
        ~Buffer();

        uint32_t top();
        void pop();
        void push(uint32_t);
        uint32_t size();
};

#endif
