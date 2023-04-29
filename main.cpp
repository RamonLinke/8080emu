#include <stdio.h>
#include <stdlib.h>

#define uint8 unsigned char
#define uint16 unsigned short

struct CPU // 8080 cpu register flags etc
{
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

    CPU() 
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

    void Reset() {
        // The reset signal forces execution of commands located at address 0x0000. The content of other processor registers is not modified.
        PC = 0x0;
    };
};

int main()
{
    CPU test = CPU();
    return 0;
}