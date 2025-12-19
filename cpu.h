#ifndef _CPU_H
#define _CPU_H

#include <math.h>
#include <functional>

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
        union {
            struct {
                bool C : 1;  // set if the last addition operation resulted in a carry or if the last subtraction operation required a borrow
                const bool V : 1;  // always 1
                bool P : 1;  // set if the number of 1 bits in the result is even.
                bool A : 1;  // set if a carry or borrow has been generated out of the least significant four bits of the accumulator
                const bool K : 1;  // always 0
                bool Z : 1;  // set if the result is zero.
                bool S : 1;  // set if the result is negative.
            };

            uint8 raw;
        };
    };

    CPU();

    void Tick(Memory* mem);

    void Halt();
    void Reset();
    void Clear();
    void SetPortOutHandler(std::function<void(uint8 port, uint8 data)> func);
    void SetPortInHandler(std::function<uint8(uint8 port)> func);

private:

    Flags flags;
    bool interrupts;
    bool halted;

    // external functions for writing and reading from the ports
    std::function<void(uint8, uint8)> port_out;
    std::function<uint8(uint8)> port_in;

    // null port functions
    static void NullPortOut(uint8, uint8);
    static uint8 NullPortIn(uint8);

    uint8 ReadPCByte(Memory* mem);
    uint16 ReadPCWord(Memory* mem);
    uint16 PopSPWord(Memory* mem);

    void PushSPWord(Memory* mem, uint16* data);

    void SetFlags(uint8 num);

    void ILL(Memory* mem);

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
    void ADD_R(uint8* reg);
    void ADC_R(uint8* reg);
    void SUB_R(uint8* reg);
    void SBB_R(uint8* reg);
    void ANA_R(uint8* reg);
    void XRA_R(uint8* reg);
    void ORA_R(uint8* reg);
    void CMP_R(uint8* reg);
    void POP_R(Memory* mem, uint16* reg);
    void PUSH_R(Memory* mem, uint16* reg);
    void RST(Memory* mem, uint8 num);

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
    void MVI_M(Memory* mem);
    void STC(Memory* mem);
    void DAD_SP(Memory* mem) { DAD(&SP); };
    void LDA(Memory* mem);
    void DCX_SP(Memory* mem) { DCX(&SP); };
    void INR_A(Memory* mem) { INR(&A); };
    void DCR_A(Memory* mem) { DCR(&A); };
    void MVI_A(Memory* mem) { MVI(mem, &A); };
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

    // 0x60
    void MOV_HB(Memory* mem) { MOV_RR(&H, &B); }
    void MOV_HC(Memory* mem) { MOV_RR(&H, &C); }
    void MOV_HD(Memory* mem) { MOV_RR(&H, &D); }
    void MOV_HE(Memory* mem) { MOV_RR(&H, &E); }
    void MOV_HH(Memory* mem) { MOV_RR(&H, &H); }
    void MOV_HL(Memory* mem) { MOV_RR(&H, &L); }
    void MOV_HM(Memory* mem) { MOV_MR(mem, &H); }
    void MOV_HA(Memory* mem) { MOV_RR(&H, &A); }
    void MOV_LB(Memory* mem) { MOV_RR(&L, &B); }
    void MOV_LC(Memory* mem) { MOV_RR(&L, &C); }
    void MOV_LD(Memory* mem) { MOV_RR(&L, &D); }
    void MOV_LE(Memory* mem) { MOV_RR(&L, &E); }
    void MOV_LH(Memory* mem) { MOV_RR(&L, &H); }
    void MOV_LL(Memory* mem) { MOV_RR(&L, &L); }
    void MOV_LM(Memory* mem) { MOV_MR(mem, &L); }
    void MOV_LA(Memory* mem) { MOV_RR(&L, &A); }

    // 0x70
    void MOV_MB(Memory* mem) { MOV_RM(mem, &B); }
    void MOV_MC(Memory* mem) { MOV_RM(mem, &C); }
    void MOV_MD(Memory* mem) { MOV_RM(mem, &D); }
    void MOV_ME(Memory* mem) { MOV_RM(mem, &E); }
    void MOV_MH(Memory* mem) { MOV_RM(mem, &H); }
    void MOV_ML(Memory* mem) { MOV_RM(mem, &L); }
    void HLT(Memory* mem);
    void MOV_MA(Memory* mem) { MOV_RM(mem, &A); }
    void MOV_AB(Memory* mem) { MOV_RR(&A, &B); }
    void MOV_AC(Memory* mem) { MOV_RR(&A, &C); }
    void MOV_AD(Memory* mem) { MOV_RR(&A, &D); }
    void MOV_AE(Memory* mem) { MOV_RR(&A, &E); }
    void MOV_AH(Memory* mem) { MOV_RR(&A, &H); }
    void MOV_AL(Memory* mem) { MOV_RR(&A, &L); }
    void MOV_AM(Memory* mem) { MOV_MR(mem, &A); }
    void MOV_AA(Memory* mem) { MOV_RR(&A, &A); }

    // 0x80
    void ADD_B(Memory* mem) { ADD_R(&B); }
    void ADD_C(Memory* mem) { ADD_R(&C); }
    void ADD_D(Memory* mem) { ADD_R(&D); }
    void ADD_E(Memory* mem) { ADD_R(&E); }
    void ADD_H(Memory* mem) { ADD_R(&H); }
    void ADD_L(Memory* mem) { ADD_R(&L); }
    void ADD_M(Memory* mem);
    void ADD_A(Memory* mem) { ADD_R(&A); }
    void ADC_B(Memory* mem) { ADC_R(&B); }
    void ADC_C(Memory* mem) { ADC_R(&C); }
    void ADC_D(Memory* mem) { ADC_R(&D); }
    void ADC_E(Memory* mem) { ADC_R(&E); }
    void ADC_H(Memory* mem) { ADC_R(&H); }
    void ADC_L(Memory* mem) { ADC_R(&L); }
    void ADC_M(Memory* mem);
    void ADC_A(Memory* mem) { ADC_R(&A); }

    // 0x90
    void SUB_B(Memory* mem) { SUB_R(&B); }
    void SUB_C(Memory* mem) { SUB_R(&C); }
    void SUB_D(Memory* mem) { SUB_R(&D); }
    void SUB_E(Memory* mem) { SUB_R(&E); }
    void SUB_H(Memory* mem) { SUB_R(&H); }
    void SUB_L(Memory* mem) { SUB_R(&L); }
    void SUB_M(Memory* mem);
    void SUB_A(Memory* mem) { SUB_R(&A); }
    void SBB_B(Memory* mem) { SBB_R(&B); }
    void SBB_C(Memory* mem) { SBB_R(&C); }
    void SBB_D(Memory* mem) { SBB_R(&D); }
    void SBB_E(Memory* mem) { SBB_R(&E); }
    void SBB_H(Memory* mem) { SBB_R(&H); }
    void SBB_L(Memory* mem) { SBB_R(&L); }
    void SBB_M(Memory* mem);
    void SBB_A(Memory* mem) { SBB_R(&A); }

    // 0xA0
    void ANA_B(Memory* mem) { ANA_R(&B); }
    void ANA_C(Memory* mem) { ANA_R(&C); }
    void ANA_D(Memory* mem) { ANA_R(&D); }
    void ANA_E(Memory* mem) { ANA_R(&E); }
    void ANA_H(Memory* mem) { ANA_R(&H); }
    void ANA_L(Memory* mem) { ANA_R(&L); }
    void ANA_M(Memory* mem);
    void ANA_A(Memory* mem) { ANA_R(&A); }
    void XRA_B(Memory* mem) { XRA_R(&B); }
    void XRA_C(Memory* mem) { XRA_R(&C); }
    void XRA_D(Memory* mem) { XRA_R(&D); }
    void XRA_E(Memory* mem) { XRA_R(&E); }
    void XRA_H(Memory* mem) { XRA_R(&H); }
    void XRA_L(Memory* mem) { XRA_R(&L); }
    void XRA_M(Memory* mem);
    void XRA_A(Memory* mem) { XRA_R(&A); }

    // 0xB0
    void ORA_B(Memory* mem) { ORA_R(&B); }
    void ORA_C(Memory* mem) { ORA_R(&C); }
    void ORA_D(Memory* mem) { ORA_R(&D); }
    void ORA_E(Memory* mem) { ORA_R(&E); }
    void ORA_H(Memory* mem) { ORA_R(&H); }
    void ORA_L(Memory* mem) { ORA_R(&L); }
    void ORA_M(Memory* mem);
    void ORA_A(Memory* mem) { ORA_R(&A); }
    void CMP_B(Memory* mem) { CMP_R(&B); }
    void CMP_C(Memory* mem) { CMP_R(&C); }
    void CMP_D(Memory* mem) { CMP_R(&D); }
    void CMP_E(Memory* mem) { CMP_R(&E); }
    void CMP_H(Memory* mem) { CMP_R(&H); }
    void CMP_L(Memory* mem) { CMP_R(&L); }
    void CMP_M(Memory* mem);
    void CMP_A(Memory* mem) { CMP_R(&A); }

    // 0xC0
    void RNZ(Memory* mem);
    void POP_B(Memory* mem) { POP_R(mem, &BC); }
    void JNZ(Memory* mem);
    void JMP(Memory* mem);
    void CNZ(Memory* mem);
    void PUSH_B(Memory* mem) { PUSH_R(mem, &BC); }
    void ADI(Memory* mem);
    void RST0(Memory* mem) { RST(mem, 0); }
    void RZ(Memory* mem);
    void RET(Memory* mem);
    void JZ(Memory* mem);
    void CZ(Memory* mem);
    void CALL(Memory* mem);
    void ACI(Memory* mem);
    void RST1(Memory* mem) { RST(mem, 1); }

    // 0xD0
    void RNC(Memory* mem);
    void POP_D(Memory* mem) { POP_R(mem, &DE); }
    void JNC(Memory* mem);
    void CNC(Memory* mem);
    void PUSH_D(Memory* mem) { PUSH_R(mem, &DE); }
    void SUI(Memory* mem);
    void RST2(Memory* mem) { RST(mem, 2); }
    void RC(Memory* mem);
    void JC(Memory* mem);
    void IN(Memory* mem);
    void CC(Memory* mem);
    void SBI(Memory* mem);
    void RST3(Memory* mem) { RST(mem, 3); }
    
    // 0xE0
    void RPO(Memory* mem);
    void POP_H(Memory* mem) { POP_R(mem, &HL); }
    void JPO(Memory* mem);
    void CPO(Memory* mem);
    void PUSH_H(Memory* mem) { PUSH_R(mem, &HL); }
    void ANI(Memory* mem);
    void RST4(Memory* mem) { RST(mem, 4); }
    void OUT(Memory* mem);
    void RPE(Memory* mem);
    void PCHL(Memory* mem);
    void JPE(Memory* mem);
    void XCHG(Memory* mem);
    void CPE(Memory* mem);
    void XRI(Memory* mem);
    void RST5(Memory* mem) { RST(mem, 5); }
    
    // 0xF0
    void RP(Memory* mem);
    void POP_PSW(Memory* mem);
    void JP(Memory* mem);
    void DI(Memory* mem);
    void CP(Memory* mem);
    void PUSH_PSW(Memory* mem);
    void ORI(Memory* mem);
    void RST6(Memory* mem) { RST(mem, 6); }
    void RM(Memory* mem);
    void SPHL(Memory* mem);
    void JM(Memory* mem);
    void EI(Memory* mem);
    void CM(Memory* mem);
    void CPI(Memory* mem);
    void RST7(Memory* mem) { RST(mem, 7); }

    struct CPUOpcode
    {
        void (CPU::*handler)(Memory* data);
    };

    // Holds the addresses to opcode functions
    CPUOpcode opcodeTable[0x100] = {
    //  0x00          0x01          0x02          0x03          0x04          0x05          0x06          0x07          0x08          0x09          0x0A          0x0B          0x0C          0x0D          0x0E          0x0F
        &CPU::NOP,    &CPU::LXI_B,  &CPU::STAX_B, &CPU::INX_B,  &CPU::INR_B,  &CPU::DCR_B,  &CPU::MVI_B,  &CPU::RLC,    &CPU::ILL,    &CPU::DAD_B,  &CPU::LDAX_B, &CPU::DCX_B,  &CPU::INR_C,  &CPU::DCR_C,  &CPU::MVI_C,  &CPU::RRC,
    //  0x10          0x11          0x12          0x13          0x14          0x15          0x16          0x17          0x18          0x19          0x1A          0x1B          0x1C          0x1D          0x1E          0x1F
        &CPU::ILL,    &CPU::LXI_D,  &CPU::STAX_D, &CPU::INX_D,  &CPU::INR_D,  &CPU::DCR_D,  &CPU::MVI_D,  &CPU::RAL,    &CPU::ILL,    &CPU::DAD_D,  &CPU::LDAX_D, &CPU::DCX_D,  &CPU::INR_E,  &CPU::DCR_E,  &CPU::MVI_E,  &CPU::RAR,
    //  0x20          0x21          0x22          0x23          0x24          0x25          0x26          0x27          0x28          0x29          0x2A          0x2B          0x2C          0x2D          0x2E          0x2F
        &CPU::ILL,    &CPU::LXI_H,  &CPU::SHLD,   &CPU::INX_H,  &CPU::INX_H,  &CPU::DCR_H,  &CPU::MVI_H,  &CPU::DAA,    &CPU::ILL,    &CPU::DAD_H,  &CPU::LHLD,   &CPU::DCX_H,  &CPU::INR_L,  &CPU::DCR_L,  &CPU::MVI_L,  &CPU::CMA,
    //  0x30          0x31          0x32          0x33          0x34          0x35          0x36          0x37          0x38          0x39          0x3A          0x3B          0x3C          0x3D          0x3E          0x3F
        &CPU::ILL,    &CPU::LXI_SP, &CPU::STA,    &CPU::INR_M,  &CPU::DCR_M,  &CPU::DCR_M,  &CPU::MVI_M,  &CPU::STC,    &CPU::ILL,    &CPU::DAD_SP, &CPU::LDA,    &CPU::DCX_SP, &CPU::INR_A,  &CPU::DCR_A,  &CPU::MVI_A,  &CPU::CMC,
    //  0x40          0x41          0x42          0x43          0x44          0x45          0x46          0x47          0x48          0x49          0x4A          0x4B          0x4C          0x4D          0x4E          0x4F
        &CPU::MOV_BB, &CPU::MOV_BC, &CPU::MOV_BD, &CPU::MOV_BE, &CPU::MOV_BH, &CPU::MOV_BL, &CPU::MOV_BM, &CPU::MOV_BA, &CPU::MOV_CB, &CPU::MOV_CC, &CPU::MOV_CD, &CPU::MOV_CE, &CPU::MOV_CH, &CPU::MOV_CL, &CPU::MOV_CM, &CPU::MOV_CA,
    //  0x50          0x51          0x52          0x53          0x54          0x55          0x56          0x57          0x58          0x59          0x5A          0x5B          0x5C          0x5D          0x5E          0x5F
        &CPU::MOV_DB, &CPU::MOV_DC, &CPU::MOV_DD, &CPU::MOV_DE, &CPU::MOV_DH, &CPU::MOV_DL, &CPU::MOV_DM, &CPU::MOV_DA, &CPU::MOV_EB, &CPU::MOV_EC, &CPU::MOV_ED, &CPU::MOV_EE, &CPU::MOV_EH, &CPU::MOV_EL, &CPU::MOV_EM, &CPU::MOV_EA,
    //  0x60          0x61          0x62          0x63          0x64          0x65          0x66          0x67          0x68          0x69          0x6A          0x6B          0x6C          0x6D          0x6E          0x6F
        &CPU::MOV_HB, &CPU::MOV_HC, &CPU::MOV_HD, &CPU::MOV_HE, &CPU::MOV_HH, &CPU::MOV_HL, &CPU::MOV_HM, &CPU::MOV_HA, &CPU::MOV_LB, &CPU::MOV_LC, &CPU::MOV_LD, &CPU::MOV_LE, &CPU::MOV_LH, &CPU::MOV_LL, &CPU::MOV_LM, &CPU::MOV_LA,
    //  0x70          0x71          0x72          0x73          0x74          0x75          0x76          0x77          0x78          0x79          0x7A          0x7B          0x7C          0x7D          0x7E          0x7F
        &CPU::MOV_MB, &CPU::MOV_MC, &CPU::MOV_MD, &CPU::MOV_ME, &CPU::MOV_MH, &CPU::MOV_ML, &CPU::HLT,    &CPU::MOV_MA, &CPU::MOV_AB, &CPU::MOV_AC, &CPU::MOV_AD, &CPU::MOV_AE, &CPU::MOV_AH, &CPU::MOV_AL, &CPU::MOV_AM, &CPU::MOV_AA,
    //  0x80          0x81          0x82          0x83          0x84          0x85          0x86          0x87          0x88          0x89          0x8A          0x8B          0x8C          0x8D          0x8E          0x8F
        &CPU::ADD_B,  &CPU::ADD_C,  &CPU::ADD_D,  &CPU::ADD_E,  &CPU::ADD_H,  &CPU::ADD_L,  &CPU::ADD_M,  &CPU::ADD_A,  &CPU::ADC_B,  &CPU::ADC_C,  &CPU::ADC_D,  &CPU::ADC_E,  &CPU::ADC_H,  &CPU::ADC_L,  &CPU::ADC_M,  &CPU::ADC_A,
    //  0x90          0x91          0x92          0x93          0x94          0x95          0x96          0x97          0x98          0x99          0x9A          0x9B          0x9C          0x9D          0x9E          0x9F
        &CPU::SUB_B,  &CPU::SUB_C,  &CPU::SUB_D,  &CPU::SUB_E,  &CPU::SUB_H,  &CPU::SUB_L,  &CPU::SUB_M,  &CPU::SUB_A,  &CPU::SBB_B,  &CPU::SBB_C,  &CPU::SBB_D,  &CPU::SBB_E,  &CPU::SBB_H,  &CPU::SBB_L,  &CPU::SBB_M,  &CPU::SBB_A,
    //  0xA0          0xA1          0xA2          0xA3          0xA4          0xA5          0xA6          0xA7          0xA8          0xA9          0xAA          0xAB          0xAC          0xAD          0xAE          0xAF
        &CPU::ANA_B,  &CPU::ANA_C,  &CPU::ANA_D,  &CPU::ANA_E,  &CPU::ANA_H,  &CPU::ANA_L,  &CPU::ANA_M,  &CPU::ANA_A,  &CPU::XRA_B,  &CPU::XRA_C,  &CPU::XRA_D,  &CPU::XRA_E,  &CPU::XRA_H,  &CPU::XRA_L,  &CPU::XRA_M,  &CPU::XRA_A,
    //  0xB0          0xB1          0xB2          0xB3          0xB4          0xB5          0xB6          0xB7          0xB8          0xB9          0xBA          0xBB          0xBC          0xBD          0xBE          0xBF
        &CPU::ORA_B,  &CPU::ORA_C,  &CPU::ORA_D,  &CPU::ORA_E,  &CPU::ORA_H,  &CPU::ORA_L,  &CPU::ORA_M,  &CPU::ORA_A,  &CPU::CMP_B,  &CPU::CMP_C,  &CPU::CMP_D,  &CPU::CMP_E,  &CPU::CMP_H,  &CPU::CMP_L,  &CPU::CMP_M,  &CPU::CMP_A,
    //  0xC0          0xC1          0xC2          0xC3          0xC4          0xC5          0xC6          0xC7          0xC8          0xC9          0xCA          0xCB          0xCC          0xCD          0xCE          0xCF
        &CPU::RNZ,    &CPU::POP_B,  &CPU::JNZ,    &CPU::JMP,    &CPU::CNZ,    &CPU::PUSH_B, &CPU::ADI,    &CPU::RST0,   &CPU::RZ,     &CPU::RET,    &CPU::JZ,     &CPU::ILL,    &CPU::CZ,     &CPU::CALL,   &CPU::ACI,    &CPU::RST1,
    //  0xD0          0xD1          0xD2          0xD3          0xD4          0xD5          0xD6          0xD7          0xD8          0xD9          0xDA          0xDB          0xDC          0xDD          0xDE          0xDF
        &CPU::RNC,    &CPU::POP_D,  &CPU::JNC,    &CPU::OUT,    &CPU::CNC,    &CPU::PUSH_D, &CPU::SUI,    &CPU::RST2,   &CPU::RC,     &CPU::ILL,    &CPU::JC,     &CPU::IN,     &CPU::CC,     &CPU::ILL,    &CPU::SBI,    &CPU::RST3,
    //  0xE0          0xE1          0xE2          0xE3          0xE4          0xE5          0xE6          0xE7          0xE8          0xE9          0xEA          0xEB          0xEC          0xED          0xEE          0xEF
        &CPU::RPO,    &CPU::POP_H,  &CPU::JPO,    &CPU::ILL,    &CPU::CPO,    &CPU::PUSH_H, &CPU::ANI,    &CPU::RST4,   &CPU::RPE,    &CPU::PCHL,   &CPU::JPE,    &CPU::XCHG,   &CPU::CPE,    &CPU::ILL,    &CPU::XRI,    &CPU::RST5,
    //  0xF0          0xF1          0xF2          0xF3          0xF4          0xF5          0xF6          0xF7          0xF8          0xF9          0xFA          0xFB          0xFC          0xFD          0xFE          0xFF
        &CPU::RP,     &CPU::POP_PSW,&CPU::JP,     &CPU::DI,     &CPU::CP,     &CPU::PUSH_PSW,&CPU::ORI,   &CPU::RST6,   &CPU::RM,     &CPU::SPHL,   &CPU::JM,     &CPU::EI,     &CPU::CM,     &CPU::ILL,    &CPU::CPI,    &CPU::RST7,
    };
};

#endif