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

    uint8 ReadPCByte(Memory* mem);
    uint16 ReadPCWord(Memory* mem);

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
    void LXI_D(Memory* mem) { LXI(mem, &DE); };
    void STAX_D(Memory* mem) { STAX(mem, &DE); };
    void INX_D(Memory* mem) { INX(mem, &DE); };
    void INR_D(Memory* mem) { INR(mem, &D); };
    void DCR_D(Memory* mem) { DCR(mem, &D); };
    void MVI_D(Memory* mem) { MVI(mem, &D); };
    void RAL(Memory* mem);
    void DAD_D(Memory* mem) { DAD(mem, &DE); };
    void LDAX_D(Memory* mem) { LDAX(mem, &DE); };
    void DCX_D(Memory* mem) { DCX(mem, &DE); };
    void INR_E(Memory* mem) { INR(mem, &E); };
    void DCR_E(Memory* mem) { DCR(mem, &E); };
    void MVI_E(Memory* mem) { MVI(mem, &E); };
    void RAR(Memory* mem);

    // 0x20
    void LXI_H(Memory* mem) { LXI(mem, &HL); };
    void SHLD(Memory* mem);
    void INX_H(Memory* mem) { INX(mem, &HL); };
    void INR_H(Memory* mem) { INR(mem, &H); };
    void DCR_H(Memory* mem) { DCR(mem, &H); };
    void MVI_H(Memory* mem) { MVI(mem, &H); };
    void DAA(Memory* mem);


    struct CPUOpcode
    {
        void (CPU::*handler)(Memory* data);
    };

    CPUOpcode opcodeRegister[0xFF] = {
        {&CPU::NOP},        // 0x00
        {&CPU::LXI_B},      // 0x01
        {&CPU::STAX_B},     // 0x02
        {&CPU::INX_B},      // 0x03
        {&CPU::INR_B},      // 0x04
        {&CPU::DCR_B},      // 0x05
        {&CPU::MVI_B},      // 0x06
        {&CPU::RLC},        // 0x07
        {&CPU::NOP},        // 0x08
        {&CPU::DAD_B},      // 0x09
        {&CPU::LDAX_B},     // 0x0A
        {&CPU::DCX_B},      // 0x0B
        {&CPU::INR_C},      // 0x0C
        {&CPU::DCR_C},      // 0x0D
        {&CPU::MVI_C},      // 0x0E
        {&CPU::RRC},        // 0x0F
        {&CPU::NOP},        // 0x10
        {&CPU::LXI_D},      // 0x11
        {&CPU::STAX_D},     // 0x12
        {&CPU::INX_D},      // 0x13
        {&CPU::INR_D},      // 0x14
        {&CPU::DCR_D},      // 0x15
        {&CPU::MVI_D},      // 0x16
        {&CPU::RAL},        // 0x17
        {&CPU::NOP},        // 0x18
        {&CPU::DAD_D},      // 0x19
        {&CPU::LDAX_D},     // 0x1A
        {&CPU::DCX_D},      // 0x1B
        {&CPU::INR_E},      // 0x1C
        {&CPU::DCR_E},      // 0x1D
        {&CPU::MVI_E},      // 0x1E
        {&CPU::RAR},        // 0x1F
        {&CPU::NOP},        // 0x20
        {&CPU::LXI_H},      // 0x21
        {&CPU::SHLD},       // 0x22
        {&CPU::INX_H},      // 0x23
        {&CPU::INX_H},      // 0x24
        {&CPU::DCR_H},      // 0x25
        {&CPU::MVI_H},      // 0x26
        {&CPU::DAA},        // 0x27

    };
};

#endif