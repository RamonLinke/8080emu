#include <stdio.h>
#include <cstring>

#include "emulator.h"

CPU::CPU()
{
    // on init, set all registers and flags to 0
    // general registers
    A = 0x0;
    BC = 0x0;
    DE = 0x0;

    // special registers
    HL = 0x0;       // indirect access
    PC = 0x0;       // program counter
    SP = 0x0;       // stack pointer

    // flags
    flags.S = 0x0;  // signed
    flags.Z = 0x0;  // zero
    flags.A = 0x0;  // aux carry
    flags.P = 0x0;  // parity
    flags.C = 0x0;  // carry
    //flags.AC = 0x0; // auxiliary carry
}

void CPU::Reset()
{
    // The reset signal forces execution of commands located at address 0x0000. The content of other processor registers is not modified.
    PC = 0x0;
};

void CPU::Tick(Memory* mem)
{
    // grab opcodeId from the memory
    uint8 opcodeId = mem->data[PC];

    // get the opcode and other info from the register
    CPUOpcode &opcode = opcodeRegister[opcodeId];

    // jump to the handler function
    (this->*opcode.handler)(mem);

    // print debug info
    printf("0x%04X - 0x%02X - %s\n", PC, opcodeId, opcode.name);
}

void CPU::SetFlags(uint8 num)
{
    // update cpu flags based on num
    flags.S = num & 0b10000000; // signed flag
    flags.Z = num == 0;         // zero flag
    flags.A = num & 0b00001000; // aux carry

    // calculate parity
    uint8 parityBits = 0;
    for (int i = 0; i < 8; i++)
        parityBits += ((num >> i) & 1);

    flags.P = (parityBits & 1) == 0;
}

// multi register cpu functions
void CPU::NOP(Memory* mem)
{
    // do nothing apart from incrementing the Program Counter
    PC += 0x01;
}

void CPU::LXI(Memory* mem, uint16* reg)
{
    // Store the data at the specified address into the register pair
    *reg = (uint16)mem->data[PC + 0x1];

    // increment the Program Counter
    PC += 0x03;
}

void CPU::STAX(Memory* mem, uint16* reg)
{
    // Store A in memory address in register pair
    uint16 memPtr = *reg;
    mem->data[memPtr] = A;

    // increment the Program Counter
    PC += 0x01;
}

void CPU::INX(Memory* mem, uint16* reg)
{
    // Increment register pair
    (*reg)++;

    // increment the Program Counter
    PC += 0x01;
}


// opcodes
void CPU::LXI_B(Memory* mem)
{
    // Store the data at the specified address into the register pair BC
    LXI(mem, &BC);
}

void CPU::STAX_B(Memory* mem)
{
    // Store A in memory address in register pair BC
    STAX(mem, &BC);
}

void CPU::INX_B(Memory* mem)
{
    // Increment register pair
    INX(mem, &BC);
}

void CPU::INR_B(Memory* mem)
{
    // Increment Register
    B++;
    SetFlags(B);

    // increment the Program Counter
    PC += 0x01;
}

void CPU::DCR_B(Memory* mem)
{
    // Decrement Register
    B--;
    SetFlags(B);

    // increment the Program Counter
    PC += 0x01;
}

void CPU::MVI_B(Memory* mem)
{    
    // Set B to operand (immediate)
    uint8 operand = mem->data[PC + 0x1];
    B = operand;

    // increment the Program Counter
    PC += 0x02;
}

void CPU::RLC(Memory* mem)
{
    // circular shift left
    A = (A << 1) + (A >> 7);

    // increment the Program Counter
    PC += 0x01;
}

void CPU::DAD_B(Memory* mem)
{
    // circular shift left
    A = (A << 1) + (A >> 7);

    // increment the Program Counter
    PC += 0x01;
}

int main()
{
    CPU* cpu = new CPU();
    Memory* memory = new Memory();

    uint16 i = 0;
    memory->data[i++] = 0x00;
    memory->data[i++] = 0x00;
    memory->data[i++] = 0x03;
    while (1) 
    {
        cpu->Tick(memory);
    }

    delete cpu;
    delete memory;

    return 0;
}