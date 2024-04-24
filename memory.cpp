#include <stdio.h>
#include <cstring>

#include "memory.h"

Memory::Memory()
{
    // clear the memory on Init
    Clear();
}

void Memory::Clear()
{
    // Set the full address space to 0
    std::memset(data, 0, sizeof(data));
}