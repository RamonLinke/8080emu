#ifndef _EMULATOR_H
#define _EMULATOR_H

#define uint8 unsigned char
#define uint16 unsigned short

struct Memory
{
    // full space of a 16 bit cpu
    uint8 data[65536];

    Memory() 
    {
        // clear the memory on Init
        Clear();
    }

    void Clear()
    {
        // Set the full address space to 0
        std::memset(data, 0, sizeof(data));
    }
};

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
        bool P;  // set if the number of 1 bits in the result is even.
        bool C;  // set if the last addition operation resulted in a carry or if the last subtraction operation required a borrow
        bool AC; // used for binary-coded decimal arithmetic (BCD).
    };

    Flags flags;

    CPU();    

    void Reset();
    void Tick(Memory* memory);

    // opcode functions
    void NopOpcode(Memory* memory); // 0x00
    void LXI_B(Memory* memory);     // 0x01

    struct CPUOpcode
    {
        uint8 opcode;
        char const* name;
        void (CPU::*handler)(Memory* data);
    };

    CPUOpcode opcodeRegister[0xFF] = {
        {0x00, "NOP", &CPU::NopOpcode},
        {0x01, "LXI B", &CPU::LXI_B},
    };
};

#endif