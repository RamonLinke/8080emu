#include <stdio.h>
#include <cstring>

#include "emulator.h"

CPU::CPU()
{
    // on init, set all registers and flags to 0
    A = 0x0;
    BC = 0x0;
    DE = 0x0;
    HL = 0x0;
    PC = 0x0;
    SP = 0x0;
    flags.S = 0x0;
    flags.Z = 0x0;
    flags.P = 0x0;
    flags.C = 0x0;
    flags.AC = 0x0;
}

void CPU::Reset()
{
    // The reset signal forces execution of commands located at address 0x0000. The content of other processor registers is not modified.
    PC = 0x0;
};

void CPU::Tick(Memory* memory)
{
    // grab opcodeId from the memory
    uint8 opcodeId = memory->data[PC];

    // get the opcode and other info from the register
    CPUOpcode &opcode = opcodeRegister[opcodeId];

    // jump to the handler function
    (this->*opcode.handler)(memory);

    // print debug info
    printf("0x%04X - 0x%02X - %s\n", PC, opcodeId, opcode.name);

    // increment our Program Counter Register
    PC += opcode.size;
}

void CPU::NopOpcode(Memory* memory)
{
    // do nothing
}

int main()
{
    CPU cpu = CPU();
    Memory mem = Memory();
    while (1) 
    {
        cpu.Tick(&mem);
    }
    return 0;
}