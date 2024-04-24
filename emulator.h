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
    void Tick(Memory* mem);

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
    void NOP(Memory* mem);          // 0x00, 0x08
    void LXI_B(Memory* mem);        // 0x01
    void STAX_B(Memory* mem);       // 0x02
    void INX_B(Memory* mem);        // 0x03
    void INR_B(Memory* mem);        // 0x04
    void DCR_B(Memory* mem);        // 0x05
    void MVI_B(Memory* mem);        // 0x06
    void RLC(Memory* mem);          // 0x07
    // NOP                          // 0x08
    void DAD_B(Memory* mem);        // 0x09
    void LDAX_B(Memory* mem);       // 0x0A
    void DCX_B(Memory* mem);        // 0x0B
    void INR_C(Memory* mem);        // 0x0C
    void DCR_C(Memory* mem);        // 0x0D
    void MVI_C(Memory* mem);        // 0x0E
    void RRC(Memory* mem);          // 0x0F



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