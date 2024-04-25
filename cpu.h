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
    void INX(uint16* reg);
    void INR(uint8* reg);
    void DCR(uint8* reg);
    void MVI(Memory* mem, uint8* reg);
    void DAD(uint16* reg);
    void LDAX(Memory* mem, uint16* reg);
    void DCX(uint16* reg);
    void MOV_RR(uint8* fromReg, uint8* toReg);
    void MOV_RM(Memory* mem, uint8* reg);
    void MOV_MR(Memory* mem, uint8* reg);

    // opcode functions
    void NOP(Memory* mem);
    void LXI_B(Memory* mem) { LXI(mem, &BC); };
    void STAX_B(Memory* mem) { STAX(mem, &BC); };
    void INX_B(Memory* mem) { INX(&BC); };
    void INR_B(Memory* mem) { INR(&B); };
    void DCR_B(Memory* mem) { DCR(&B); };
    void MVI_B(Memory* mem) { MVI(mem, &B); };
    void RLC(Memory* mem);
    void DAD_B(Memory* mem) { DAD(&BC); };
    void LDAX_B(Memory* mem) { LDAX(mem, &BC); };
    void DCX_B(Memory* mem) { DCX(&BC); };
    void INR_C(Memory* mem) { INR(&C); };
    void DCR_C(Memory* mem) { DCR(&C); };
    void MVI_C(Memory* mem) { MVI(mem, &C); };
    void RRC(Memory* mem);

    // 0x10
    void LXI_D(Memory* mem) { LXI(mem, &DE); };
    void STAX_D(Memory* mem) { STAX(mem, &DE); };
    void INX_D(Memory* mem) { INX(&DE); };
    void INR_D(Memory* mem) { INR(&D); };
    void DCR_D(Memory* mem) { DCR(&D); };
    void MVI_D(Memory* mem) { MVI(mem, &D); };
    void RAL(Memory* mem);
    void DAD_D(Memory* mem) { DAD(&DE); };
    void LDAX_D(Memory* mem) { LDAX(mem, &DE); };
    void DCX_D(Memory* mem) { DCX(&DE); };
    void INR_E(Memory* mem) { INR(&E); };
    void DCR_E(Memory* mem) { DCR(&E); };
    void MVI_E(Memory* mem) { MVI(mem, &E); };
    void RAR(Memory* mem);

    // 0x20
    void LXI_H(Memory* mem) { LXI(mem, &HL); };
    void SHLD(Memory* mem);
    void INX_H(Memory* mem) { INX(&HL); };
    void INR_H(Memory* mem) { INR(&H); };
    void DCR_H(Memory* mem) { DCR(&H); };
    void MVI_H(Memory* mem) { MVI(mem, &H); };
    void DAA(Memory* mem);
    void DAD_H(Memory* mem) { DAD(&HL); };
    void LHLD(Memory* mem);
    void DCX_H(Memory* mem) { DCX(&HL); };
    void INR_L(Memory* mem) { INR(&L); };
    void DCR_L(Memory* mem) { DCR(&L); };
    void MVI_L(Memory* mem) { MVI(mem, &L); };
    void CMA(Memory* mem);

    // 0x30
    void LXI_SP(Memory* mem) { LXI(mem, &SP); };
    void STA(Memory* mem);
    void INX_SP(Memory* mem) { INX(&SP); };
    void INR_M(Memory* mem);
    void DCR_M(Memory* mem);
    void MVI_M(Memory* mem) { MVI(mem, &H); };
    void STC(Memory* mem);
    void DAD_SP(Memory* mem) { DAD(&HL); };
    void LDA(Memory* mem);
    void DCX_SP(Memory* mem) { DCX(&HL); };
    void INR_A(Memory* mem) { INR(&L); };
    void DCR_A(Memory* mem) { DCR(&L); };
    void MVI_A(Memory* mem) { MVI(mem, &L); };
    void CMC(Memory* mem);

    // 0x40
    void MOV_BB(Memory* mem) { MOV_RR(&B, &B); }
    void MOV_BC(Memory* mem) { MOV_RR(&B, &C); }
    void MOV_BD(Memory* mem) { MOV_RR(&B, &D); }
    void MOV_BE(Memory* mem) { MOV_RR(&B, &E); }
    void MOV_BH(Memory* mem) { MOV_RR(&B, &H); }
    void MOV_BL(Memory* mem) { MOV_RR(&B, &L); }
    void MOV_BM(Memory* mem) { MOV_MR(mem, &B); }
    void MOV_BA(Memory* mem) { MOV_RR(&B, &A); }
    void MOV_CB(Memory* mem) { MOV_RR(&C, &B); }
    void MOV_CC(Memory* mem) { MOV_RR(&C, &C); }
    void MOV_CD(Memory* mem) { MOV_RR(&C, &D); }
    void MOV_CE(Memory* mem) { MOV_RR(&C, &E); }
    void MOV_CH(Memory* mem) { MOV_RR(&C, &H); }
    void MOV_CL(Memory* mem) { MOV_RR(&C, &L); }
    void MOV_CM(Memory* mem) { MOV_MR(mem, &C); }
    void MOV_CA(Memory* mem) { MOV_RR(&C, &A); }

    // 0x50
    void MOV_DB(Memory* mem) { MOV_RR(&D, &B); }
    void MOV_DC(Memory* mem) { MOV_RR(&D, &C); }
    void MOV_DD(Memory* mem) { MOV_RR(&D, &D); }
    void MOV_DE(Memory* mem) { MOV_RR(&D, &E); }
    void MOV_DH(Memory* mem) { MOV_RR(&D, &H); }
    void MOV_DL(Memory* mem) { MOV_RR(&D, &L); }
    void MOV_DM(Memory* mem) { MOV_MR(mem, &D); }
    void MOV_DA(Memory* mem) { MOV_RR(&D, &A); }
    void MOV_EB(Memory* mem) { MOV_RR(&E, &B); }
    void MOV_EC(Memory* mem) { MOV_RR(&E, &C); }
    void MOV_ED(Memory* mem) { MOV_RR(&E, &D); }
    void MOV_EE(Memory* mem) { MOV_RR(&E, &E); }
    void MOV_EH(Memory* mem) { MOV_RR(&E, &H); }
    void MOV_EL(Memory* mem) { MOV_RR(&E, &L); }
    void MOV_EM(Memory* mem) { MOV_MR(mem, &E); }
    void MOV_EA(Memory* mem) { MOV_RR(&E, &A); }


    struct CPUOpcode
    {
        void (CPU::*handler)(Memory* data);
    };

    CPUOpcode opcodeRegister[0xFF] = {
    //  0x00          0x01          0x02          0x03          0x04          0x05          0x06          0x07          0x08(TODO)    0x09          0x0A          0x0B          0x0C          0x0D          0x0E          0x0F
        &CPU::NOP,    &CPU::LXI_B,  &CPU::STAX_B, &CPU::INX_B,  &CPU::INR_B,  &CPU::DCR_B,  &CPU::MVI_B,  &CPU::RLC,    &CPU::NOP,    &CPU::DAD_B,  &CPU::LDAX_B, &CPU::DCX_B,  &CPU::INR_C,  &CPU::DCR_C,  &CPU::MVI_C,  &CPU::RRC,
    //  0x10(TODO)    0x11          0x12          0x13          0x14          0x15          0x16          0x17          0x18(TODO)    0x19          0x1A          0x1B          0x1C          0x1D          0x1E          0x1F
        &CPU::NOP,    &CPU::LXI_D,  &CPU::STAX_D, &CPU::INX_D,  &CPU::INR_D,  &CPU::DCR_D,  &CPU::MVI_D,  &CPU::RAL,    &CPU::NOP,    &CPU::DAD_D,  &CPU::LDAX_D, &CPU::DCX_D,  &CPU::INR_E,  &CPU::DCR_E,  &CPU::MVI_E,  &CPU::RAR, 
    //  0x20(TODO)    0x21          0x22          0x23          0x24          0x25          0x26          0x27          0x28(TODO)    0x29          0x2A          0x2B          0x2C          0x2D          0x2E          0x2F
        &CPU::NOP,    &CPU::LXI_H,  &CPU::SHLD,   &CPU::INX_H,  &CPU::INX_H,  &CPU::DCR_H,  &CPU::MVI_H,  &CPU::DAA,    &CPU::NOP,    &CPU::DAD_H,  &CPU::LHLD,   &CPU::DCX_H,  &CPU::INR_L,  &CPU::DCR_L,  &CPU::MVI_L,  &CPU::CMA, 
    //  0x30(TODO)    0x31          0x32          0x33          0x34          0x35          0x36          0x37          0x38(TODO)    0x39          0x3A          0x3B          0x3C          0x3D          0x3E          0x3F
        &CPU::NOP,    &CPU::LXI_SP, &CPU::STA,    &CPU::INR_M,  &CPU::DCR_M,  &CPU::DCR_M,  &CPU::MVI_M,  &CPU::STC,    &CPU::NOP,    &CPU::DAD_SP, &CPU::LDA,    &CPU::DCX_SP, &CPU::INR_A,  &CPU::DCR_A,  &CPU::MVI_A,  &CPU::CMC,
    //  0x40          0x41          0x42          0x43          0x44          0x45          0x46          0x47          0x48          0x49          0x4A          0x4B          0x4C          0x4D          0x4E          0x4F
        &CPU::MOV_BB, &CPU::MOV_BC, &CPU::MOV_BD, &CPU::MOV_BE, &CPU::MOV_BH, &CPU::MOV_BL, &CPU::MOV_BM, &CPU::MOV_BA, &CPU::MOV_CB, &CPU::MOV_CC, &CPU::MOV_CD, &CPU::MOV_CE, &CPU::MOV_CH, &CPU::MOV_CL, &CPU::MOV_CM, &CPU::MOV_CA,
    //  0x50          0x51          0x52          0x53          0x54          0x55          0x56          0x57          0x58          0x59          0x5A          0x5B          0x5C          0x5D          0x5E          0x5F
        &CPU::MOV_DB, &CPU::MOV_DC, &CPU::MOV_DD, &CPU::MOV_DE, &CPU::MOV_DH, &CPU::MOV_DL, &CPU::MOV_DM, &CPU::MOV_DA, &CPU::MOV_EB, &CPU::MOV_EC, &CPU::MOV_ED, &CPU::MOV_EE, &CPU::MOV_EH, &CPU::MOV_EL, &CPU::MOV_EM, &CPU::MOV_EA,
    };
};

#endif