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
}

void CPU::NopOpcode(Memory* memory)
{
    // do nothing apart from incrementing the Program Counter
    PC += 0x01;
}

void CPU::LXI_B(Memory* memory)
{
    // Store the data at the specified address into the register pair BC
    B = (uint16)memory->data[SP+0x1];
    C = (uint16)memory->data[SP+0x2];
    
    // increment the Program Counter
    PC += 0x03;
}

void CPU::STAX_B(Memory* memory)
{
    // Store A in memory address in register pair
    uint16 pointer = static_cast<unsigned>(B) << 8 | static_cast<unsigned>(C);
    memory->data[pointer] = A;

    // increment the Program Counter
    PC += 0x01;

}void CPU::INX_B(Memory* memory)
{
    // Increment register pair
    uint16 pairValue = static_cast<unsigned>(B) << 8 | static_cast<unsigned>(C);
    pairValue++;
    
    B = (uint8)(pairValue >> 8);
    C = (uint8)pairValue;

    // increment the Program Counter
    PC += 0x01;
}

int main()
{
    CPU cpu = CPU();
    Memory mem = Memory();

    uint16 i = 0;
    mem.data[i++] = 0x01;
    mem.data[i++] = 0xBA;
    mem.data[i++] = 0xD0;
    while (1) 
    {
        cpu.Tick(&mem);
    }
    return 0;
}