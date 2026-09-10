#ifndef _MEMORY_H
#define _MEMORY_H

#include <math.h>

#include "defines.h"

// full 16-bit addressable range
#define MEMORY_SIZE 0x10000

class Memory
{
public:
    Memory();
    Memory(uint8 initData[MEMORY_SIZE]);

    uint8 Read(uint16 offset);
    void Write(uint16 offset, uint8 val);
    void Clear();

private:
    uint8 data[MEMORY_SIZE];
};

#endif