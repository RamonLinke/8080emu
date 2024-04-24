#include <stdio.h>
#include <cstring>

#include "cpu.h"

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
    uint8 opcodeId = ReadPCByte(mem);

    // get the opcode and other info from the register
    CPUOpcode &opcode = opcodeRegister[opcodeId];

    // jump to the handler function
    (this->*opcode.handler)(mem);

    // print debug info
    printf("0x%04X - 0x%02X - %s\n", PC, opcodeId, "todo");
}

uint8 CPU::ReadPCByte(Memory* mem)
{
    return mem->Read(PC++);
}

uint16 CPU::ReadPCWord(Memory* mem)
{
    return mem->Read(PC++) << 8 | mem->Read(PC++);
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
    // do nothing
}

void CPU::LXI(Memory* mem, uint16* reg)
{
    // Load with immediate value B
    *reg = ReadPCWord(mem);
}

void CPU::STAX(Memory* mem, uint16* reg)
{
    // Store A in memory address in register pair
    uint16 memPtr = *reg;
    mem->Write(memPtr, A);
}

void CPU::INX(Memory* mem, uint16* reg)
{
    // Increment register pair
    (*reg)++;
}

void CPU::INR(Memory* mem, uint8* reg)
{
    // Increment Register
    (*reg)++;
    SetFlags(*reg);
}

void CPU::DCR(Memory* mem, uint8* reg)
{
    // Increment Register
    (*reg)--;
    SetFlags(*reg);
}

void CPU::MVI(Memory* mem, uint8* reg)
{
    // Load with immediate value
    *reg = ReadPCByte(mem);
}

void CPU::DAD(Memory* mem, uint16* reg)
{
    // Double Add; register pair is added to HL
    HL += *reg;
}

void CPU::LDAX(Memory* mem, uint16* reg)
{
    // Load A from memory address in register pair
    A = mem->Read(*reg);
}

void CPU::DCX(Memory* mem, uint16* reg)
{
    // Load A from memory address in register pair
    (*reg)--;
}


// unique opcodes
void CPU::RLC(Memory* mem)
{
    // Rotate A Left (Circular)
    A = (A << 1) + (A >> 7);
}

void CPU::RRC(Memory* mem)
{
    // Rotate A Right (Circular)
    A = (A >> 1) + (A << 7);
}

void CPU::RAL(Memory* mem)
{
    // Rotate A Left through carry flag
    bool carry = flags.C;
    flags.C = (A >> 7);
    A = (A << 1) | carry;
}

void CPU::RAR(Memory* mem)
{
    // Rotate A Right through carry flag
    bool carry = flags.C;
    flags.C = (A & 1);
    A = (A >> 1) | (carry << 7);
}

void CPU::SHLD(Memory* mem)
{
    // Store HL at immediate address
    mem->Write(PC, ReadPCByte(mem));
    mem->Write(PC, ReadPCByte(mem));
}

void CPU::DAA(Memory* mem)
{
    // Decimal Adjust Accumulator
    uint8 old_A = A;
    if (((A & 0x0F) > 9) || flags.A)
    {
        A = A + 6;
        flags.A = 1;
    }
    if (A > 0x99 || flags.C)
    {
        A = A + 0x60;
        flags.C = 1;
    }
}

void CPU::LHLD(Memory* mem)
{
    // Load HL from immediate address
    L = ReadPCByte(mem);
    H = ReadPCByte(mem);
}

void CPU::CMA(Memory* mem)
{
    // Complement A
    A = ~A;
}

void CPU::STA(Memory* mem)
{
    // Store A in memory
    mem->Write(ReadPCWord(mem), A);
}

void CPU::INR_M(Memory* mem)
{
    // Increment Memory at [HL]
    uint8 valPtr = mem->Read(HL);
    valPtr++;
    SetFlags(valPtr);
    mem->Write(HL, valPtr);
}

void CPU::DCR_M(Memory* mem)
{
    // Decrement Memory at [HL]
    uint8 valPtr = mem->Read(HL);
    valPtr--;
    SetFlags(valPtr);
    mem->Write(HL, valPtr);
}

void CPU::STC(Memory* mem)
{
    // Set Carry Flag
    flags.C = 1;
}

void CPU::LDA(Memory* mem)
{
    // Load A from memory
    A = mem->Read(ReadPCWord(mem));
}

void CPU::CMC(Memory* mem)
{
    // Load A from memory
    flags.C = !flags.C;
}