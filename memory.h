#ifndef _MEMORY_H
#define _MEMORY_H

#include <math.h>

#include "defines.h"

#define uint8 unsigned char
#define uint16 unsigned short

class Memory
{
public:
    Memory();

    uint8 Read(uint16 offset);
    void Write(uint16 offset, uint8 val);

private:

    // full space of a 16 bit cpu
    uint8 data[0x10000];

    void Clear();
};

#endif