#ifndef _MEMORY_H
#define _MEMORY_H

#include <math.h>

#include "defines.h"

#define uint8 unsigned char
#define uint16 unsigned short

class Memory
{
public:
    // full space of a 16 bit cpu
    uint8 data[65536];

    Memory();

private:
    void Clear();
};

#endif