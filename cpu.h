#ifndef _CPU_H
#define _CPU_H

#include <math.h>

#include "memory.h"
#include "defines.h"

class CPU // 8080 cpu register flags etc
{
public:    
    // accumulator
    uint8 A;

    // general registers:
    union
    {
        struct {
            uint8 B; // register B
            uint8 C; // register C
        };

        uint16 BC; // B and C can be combined into BC
    };

    union
    {
        struct {
            uint8 D; // register D
            uint8 E; // register E
        };

        uint16 DE; // D and E can be combined into BC
    };

    union
    {
        struct {
            uint8 H; // register H
            uint8 L; // register L
        };

        uint16 HL; // H and L can be combined into HL
    };

    // special registers:
    uint16 PC;      // Program counter
    uint16 SP;      // Stack counter

    // status registers:
    struct Flags {
        bool S;  // set if the result is negative.
        bool Z;  // set if the result is zero.
        bool A;  // set if a carry or borrow has been generated out of the least significant four bits of the accumulator
        bool P;  // set if the number of 1 bits in the result is even.
        bool C;  // set if the last addition operation resulted in a carry or if the last subtraction operation required a borrow
    };

    CPU();    

    void Reset();
    void Tick(Memory* mem);

private:

    Flags flags;

    void SetFlags(uint8 num);

    // multi register cpu functions
    void LXI(Memory* mem, uint16* reg);
    void STAX(Memory* mem, uint16* reg);
    void INX(Memory* mem, uint16* reg);
    void INR(Memory* mem, uint8* reg);
    void DCR(Memory* mem, uint8* reg);
    void MVI(Memory* mem, uint8* reg);
    void DAD(Memory* mem, uint16* reg);
    void LDAX(Memory* mem, uint16* reg);
    void DCX(Memory* mem, uint16* reg);

    // opcode functions
    void NOP(Memory* mem);
    void LXI_B(Memory* mem) { LXI(mem, &BC); };
    void STAX_B(Memory* mem) { STAX(mem, &BC); };
    void INX_B(Memory* mem) { INX(mem, &BC); };
    void INR_B(Memory* mem) { INR(mem, &B); };
    void DCR_B(Memory* mem) { DCR(mem, &B); };
    void MVI_B(Memory* mem) { MVI(mem, &B); };
    void RLC(Memory* mem);
    void DAD_B(Memory* mem) { DAD(mem, &BC); };
    void LDAX_B(Memory* mem) { LDAX(mem, &BC); };
    void DCX_B(Memory* mem) { DCX(mem, &BC); };
    void INR_C(Memory* mem) { INR(mem, &C); };
    void DCR_C(Memory* mem) { DCR(mem, &C); };
    void MVI_C(Memory* mem) { MVI(mem, &C); };
    void RRC(Memory* mem);

    // 0x10



    struct CPUOpcode
    {
        void (CPU::*handler)(Memory* data);
    };

    CPUOpcode opcodeRegister[0xFF] = {
        {&CPU::NOP},
        {&CPU::LXI_B},
        {&CPU::STAX_B},
        {&CPU::INX_B},
        {&CPU::INR_B},
        {&CPU::DCR_B},
        {&CPU::MVI_B},
        {&CPU::RLC},
        {&CPU::NOP},
        {&CPU::DAD_B},
        {&CPU::LDAX_B},
        {&CPU::DCX_B},
        {&CPU::INR_C},
        {&CPU::DCR_C},
        {&CPU::MVI_C},
        {&CPU::RRC},
    };
};

#endif