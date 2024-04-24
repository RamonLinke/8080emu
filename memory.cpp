#include <stdio.h>
#include <cstring>

#include "memory.h"

Memory::Memory()
{
    // clear the memory on Init
    Clear();
}

uint8 Memory::Read(uint16 offset)
{
    return data[offset];
}

void Memory::Write(uint16 offset, uint8 val)
{
    data[offset] = val;
}

void Memory::Clear()
{
    // Set the full address space to 0
    std::memset(data, 0, sizeof(data));
}