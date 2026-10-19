#ifndef _TEST_HELPERS_H
#define _TEST_HELPERS_H

#include <gtest/gtest.h>
#include <initializer_list>
#include <string>

#include "cpu.h"
#include "memory.h"

namespace op = CPUOpcode;

// status flag bits, layout: S Z 0 A 0 P 1 C
namespace Flag
{
    enum : uint8
    {
        CY    = 0x01, // carry
        FIXED = 0x02, // always 1
        P     = 0x04, // parity
        AC    = 0x10, // aux carry
        Z     = 0x40, // zero
        S     = 0x80, // sign
        ALL   = S | Z | AC | P | CY | FIXED,
    };
}

// register index as encoded in the opcodes (e.g. MOV is 01DDDSSS)
namespace Reg
{
    enum : int { B, C, D, E, H, L, M, A };
    inline const char* Name(int reg) { return &"B\0C\0D\0E\0H\0L\0M\0A"[reg * 2]; }
}

// register pair index as encoded in the opcodes (e.g. LXI is 00RP0001)
namespace Pair
{
    enum : int { BC, DE, HL, SP };
}

// formats flags as "0x57 [.Z.A.P1C]" so failing tests show which flag differs
inline std::string FlagString(uint8 flags)
{
    const char* names = "SZ.A.P1C";
    char buffer[32];
    int n = snprintf(buffer, sizeof(buffer), "0x%02X [", flags);
    for (int bit = 7; bit >= 0; bit--)
        buffer[n++] = (flags & (1 << bit)) ? names[7 - bit] : '.';
    buffer[n++] = ']';
    buffer[n] = 0;
    return buffer;
}

#define EXPECT_FLAGS(expected) EXPECT_EQ(FlagString(cpu.GetFlags()), FlagString(expected))

class CPUTest : public ::testing::Test
{
protected:
    // programs are loaded where CP/M loads them
    static constexpr uint16 ORIGIN = 0x0100;
    // HL points here for the memory (M) operand
    static constexpr uint16 DATA = 0x2000;

    CPU cpu;
    Memory mem;

    void ResetAll()
    {
        cpu.Clear();
        mem.Clear();
    }

    // writes the program at ORIGIN and points the PC at it
    void Load(std::initializer_list<uint8> program)
    {
        Poke(ORIGIN, program);
        cpu.PC = ORIGIN;
    }

    void Poke(uint16 address, std::initializer_list<uint8> bytes)
    {
        for (uint8 b : bytes)
            mem.Write(address++, b);
    }

    uint16 PeekWord(uint16 address)
    {
        return mem.Read(address) | (mem.Read(address + 1) << 8);
    }

    // executes one opcode, returns the amount of ticks it took
    uint8 Step() { return cpu.Step(&mem); }

    void Run(int opcodes)
    {
        for (int i = 0; i < opcodes; i++)
            Step();
    }

    // register by its opcode encoding, M is the memory at HL
    uint8& Reg(int reg)
    {
        switch (reg)
        {
            case Reg::B: return cpu.B;
            case Reg::C: return cpu.C;
            case Reg::D: return cpu.D;
            case Reg::E: return cpu.E;
            case Reg::H: return cpu.H;
            case Reg::L: return cpu.L;
            case Reg::M: return mem.Raw()[cpu.HL];
            default:     return cpu.A;
        }
    }

    // register pair by its opcode encoding
    uint16& RegPair(int pair)
    {
        switch (pair)
        {
            case Pair::BC: return cpu.BC;
            case Pair::DE: return cpu.DE;
            case Pair::HL: return cpu.HL;
            default:       return cpu.SP;
        }
    }
};

#endif
