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
        bool A;  // set if a carry or borrow has been generated out of the least significant four bits of the accumulator
        bool P;  // set if the number of 1 bits in the result is even.
        bool C;  // set if the last addition operation resulted in a carry or if the last subtraction operation required a borrow
    };

    Flags flags;

    CPU();    

    void Reset();
    void Tick(Memory* memory);

    void SetFlags(uint8 num);

    // opcode functions
    void NOP(Memory* memory);       // 0x00, 0x08
    void LXI_B(Memory* memory);     // 0x01
    void STAX_B(Memory* memory);    // 0x02
    void INX_B(Memory* memory);     // 0x03
    void INR_B(Memory* memory);     // 0x04
    void DCR_B(Memory* memory);     // 0x05
    void MVI_B(Memory* memory);     // 0x06
    void RLC(Memory* memory);       // 0x07
    // NOP
    void DAD_B(Memory* memory);     // 0x09

    struct CPUOpcode
    {
        char const* name;
        void (CPU::*handler)(Memory* data);
    };

    CPUOpcode opcodeRegister[0xFF] = {
        {"NOP", &CPU::NOP},
        {"LXI B", &CPU::LXI_B},
        {"STAX B", &CPU::STAX_B},
        {"INX B", &CPU::INX_B},
        {"INR B", &CPU::INR_B},
        {"DCR B", &CPU::DCR_B},
        {"MVI B", &CPU::MVI_B},
        {"RLC B", &CPU::RLC},
        {"NOP 8", &CPU::NOP},
    };
};

#endif