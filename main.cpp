#include "cpu.h"
#include "memory.h"

int main()
{
    CPU* cpu = new CPU();
    Memory* memory = new Memory();

    uint16 i = 0;
    memory->Write(i++, 0x00);
    memory->Write(i++, 0x00);
    memory->Write(i++, 0x03);
    while (1)
    {
        cpu->Tick(memory);
    }

    delete cpu;
    delete memory;

    return 0;
}