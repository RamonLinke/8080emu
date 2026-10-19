#include "test_helpers.h"

// =============================================================================
// data transfer
// =============================================================================

TEST_F(CPUTest, NOP)
{
    Load({op::NOP});
    cpu.SetFlags(Flag::ALL);
    Step();
    EXPECT_EQ(cpu.PC, ORIGIN + 1);
    EXPECT_FLAGS(Flag::ALL);
}

TEST_F(CPUTest, LXI_AllPairs)
{
    for (int pair = 0; pair < 4; pair++)
    {
        SCOPED_TRACE(testing::Message() << "pair " << pair);
        ResetAll();
        Load({uint8(0x01 | pair << 4), 0x34, 0x12});
        Step();
        EXPECT_EQ(RegPair(pair), 0x1234);
        EXPECT_EQ(cpu.PC, ORIGIN + 3);
    }
}

TEST_F(CPUTest, MVI_AllRegisters)
{
    for (int reg = 0; reg < 8; reg++)
    {
        SCOPED_TRACE(Reg::Name(reg));
        ResetAll();
        cpu.HL = DATA;
        cpu.SetFlags(Flag::ALL);
        Load({uint8(0x06 | reg << 3), 0xA5});
        Step();
        EXPECT_EQ(Reg(reg), 0xA5);
        EXPECT_EQ(cpu.PC, ORIGIN + 2);
        EXPECT_FLAGS(Flag::ALL);
    }
}

TEST_F(CPUTest, MOV_AllRegisterCombinations)
{
    for (int dst = 0; dst < 8; dst++)
    {
        for (int src = 0; src < 8; src++)
        {
            if (dst == Reg::M && src == Reg::M)
                continue; // that encoding is HLT

            SCOPED_TRACE(testing::Message() << "MOV " << Reg::Name(dst) << "," << Reg::Name(src));
            ResetAll();
            cpu.BC = 0x1122;
            cpu.DE = 0x3344;
            cpu.HL = DATA;
            cpu.A = 0x77;
            mem.Write(DATA, 0x99);
            cpu.SetFlags(Flag::ALL);

            uint8 expected = Reg(src);
            Load({uint8(0x40 | dst << 3 | src)});
            Step();

            EXPECT_EQ(Reg(dst), expected);
            EXPECT_EQ(cpu.PC, ORIGIN + 1);
            EXPECT_FLAGS(Flag::ALL);
        }
    }
}

TEST_F(CPUTest, STAX_StoresAAtThePairAddress)
{
    Load({op::STAX_B, op::STAX_D});
    cpu.A = 0x42;
    cpu.BC = 0x3F16;
    cpu.DE = 0x3F17;
    Run(2);
    EXPECT_EQ(mem.Read(0x3F16), 0x42);
    EXPECT_EQ(mem.Read(0x3F17), 0x42);
}

TEST_F(CPUTest, LDAX_LoadsAFromThePairAddress)
{
    Load({op::LDAX_B});
    cpu.BC = 0x938B;
    mem.Write(0x938B, 0x55);
    Step();
    EXPECT_EQ(cpu.A, 0x55);

    Load({op::LDAX_D});
    cpu.DE = 0x938C;
    mem.Write(0x938C, 0x66);
    Step();
    EXPECT_EQ(cpu.A, 0x66);
}

TEST_F(CPUTest, STA_And_LDA)
{
    Load({op::STA, 0x45, 0x23, op::LDA, 0x46, 0x23});
    mem.Write(0x2346, 0x5A);
    cpu.A = 0x99;
    Step();
    EXPECT_EQ(mem.Read(0x2345), 0x99);
    Step();
    EXPECT_EQ(cpu.A, 0x5A);
    EXPECT_EQ(cpu.PC, ORIGIN + 6);
}

TEST_F(CPUTest, SHLD_StoresLThenH)
{
    Load({op::SHLD, 0x0A, 0x30});
    cpu.HL = 0xAE29;
    Step();
    EXPECT_EQ(mem.Read(0x300A), 0x29);
    EXPECT_EQ(mem.Read(0x300B), 0xAE);
    EXPECT_EQ(cpu.PC, ORIGIN + 3);
}

TEST_F(CPUTest, LHLD_LoadsLThenH)
{
    Load({op::LHLD, 0x5B, 0x32});
    Poke(0x325B, {0xFF, 0x03});
    Step();
    EXPECT_EQ(cpu.L, 0xFF);
    EXPECT_EQ(cpu.H, 0x03);
    EXPECT_EQ(cpu.PC, ORIGIN + 3);
}

TEST_F(CPUTest, XCHG_SwapsDEAndHL)
{
    Load({op::XCHG});
    cpu.DE = 0x1234;
    cpu.HL = 0xABCD;
    Step();
    EXPECT_EQ(cpu.DE, 0xABCD);
    EXPECT_EQ(cpu.HL, 0x1234);
}

TEST_F(CPUTest, XTHL_SwapsHLWithTopOfStack)
{
    Load({op::XTHL});
    cpu.SP = 0x10AD;
    cpu.HL = 0x0B3C;
    Poke(0x10AD, {0xF0, 0x0D});
    Step();
    EXPECT_EQ(cpu.HL, 0x0DF0);
    EXPECT_EQ(mem.Read(0x10AD), 0x3C);
    EXPECT_EQ(mem.Read(0x10AE), 0x0B);
    EXPECT_EQ(cpu.SP, 0x10AD);
}

TEST_F(CPUTest, SPHL_LoadsSPFromHL)
{
    Load({op::SPHL});
    cpu.HL = 0x506C;
    Step();
    EXPECT_EQ(cpu.SP, 0x506C);
}

// =============================================================================
// stack
// =============================================================================

TEST_F(CPUTest, PUSH_AllPairs)
{
    const uint8 opcodes[] = {op::PUSH_B, op::PUSH_D, op::PUSH_H};
    for (int pair = 0; pair < 3; pair++)
    {
        SCOPED_TRACE(testing::Message() << "pair " << pair);
        ResetAll();
        cpu.SP = 0x3A2C;
        RegPair(pair) = 0x8F9D;
        Load({opcodes[pair]});
        Step();
        EXPECT_EQ(cpu.SP, 0x3A2A);
        EXPECT_EQ(mem.Read(0x3A2B), 0x8F); // high byte at SP-1
        EXPECT_EQ(mem.Read(0x3A2A), 0x9D); // low byte at SP-2
    }
}

TEST_F(CPUTest, POP_AllPairs)
{
    const uint8 opcodes[] = {op::POP_B, op::POP_D, op::POP_H};
    for (int pair = 0; pair < 3; pair++)
    {
        SCOPED_TRACE(testing::Message() << "pair " << pair);
        ResetAll();
        cpu.SP = 0x1239;
        Poke(0x1239, {0x3D, 0x93});
        Load({opcodes[pair]});
        Step();
        EXPECT_EQ(RegPair(pair), 0x933D);
        EXPECT_EQ(cpu.SP, 0x123B);
    }
}

TEST_F(CPUTest, PUSH_PSW_PushesAAndFlags)
{
    Load({op::PUSH_PSW});
    cpu.SP = 0x502A;
    cpu.A = 0x1F;
    cpu.SetFlags(Flag::Z | Flag::P | Flag::CY | Flag::FIXED);
    Step();
    EXPECT_EQ(cpu.SP, 0x5028);
    EXPECT_EQ(mem.Read(0x5029), 0x1F);
    EXPECT_EQ(mem.Read(0x5028), 0x47);
}

TEST_F(CPUTest, POP_PSW_RestoresAAndFlags)
{
    Load({op::POP_PSW});
    cpu.SP = 0x2C00;
    Poke(0x2C00, {0xD7, 0xFF});
    Step();
    EXPECT_EQ(cpu.A, 0xFF);
    EXPECT_FLAGS(Flag::ALL);
    EXPECT_EQ(cpu.SP, 0x2C02);
}

TEST_F(CPUTest, POP_PSW_KeepsTheFixedFlagBits)
{
    // bit 1 of the flags is always 1 and bits 3 and 5 are always 0 on the 8080
    Load({op::POP_PSW});
    cpu.SP = 0x2C00;
    Poke(0x2C00, {0xFF, 0x00});
    Step();
    EXPECT_FLAGS(Flag::ALL);

    Load({op::POP_PSW});
    cpu.SP = 0x2C00;
    Poke(0x2C00, {0x00, 0x00});
    Step();
    EXPECT_FLAGS(Flag::FIXED);
}

TEST_F(CPUTest, PUSH_POP_RoundTrip)
{
    Load({op::PUSH_B, op::PUSH_PSW, op::POP_H, op::POP_D});
    cpu.SP = 0x4000;
    cpu.BC = 0xBEEF;
    cpu.A = 0x12;
    cpu.SetFlags(Flag::S | Flag::CY | Flag::FIXED);
    Run(4);
    EXPECT_EQ(cpu.HL, 0x1283);
    EXPECT_EQ(cpu.DE, 0xBEEF);
    EXPECT_EQ(cpu.SP, 0x4000);
}

// =============================================================================
// increment / decrement
// =============================================================================

TEST_F(CPUTest, INX_AllPairs)
{
    for (int pair = 0; pair < 4; pair++)
    {
        SCOPED_TRACE(testing::Message() << "pair " << pair);
        ResetAll();
        Load({uint8(0x03 | pair << 4), uint8(0x03 | pair << 4)});
        RegPair(pair) = 0x38FF;
        cpu.SetFlags(Flag::FIXED);
        Step();
        EXPECT_EQ(RegPair(pair), 0x3900);

        // wraps around without setting any flags
        RegPair(pair) = 0xFFFF;
        Step();
        EXPECT_EQ(RegPair(pair), 0x0000);
        EXPECT_FLAGS(Flag::FIXED);
    }
}

TEST_F(CPUTest, DCX_AllPairs)
{
    for (int pair = 0; pair < 4; pair++)
    {
        SCOPED_TRACE(testing::Message() << "pair " << pair);
        ResetAll();
        Load({uint8(0x0B | pair << 4), uint8(0x0B | pair << 4)});
        RegPair(pair) = 0x9800;
        cpu.SetFlags(Flag::FIXED);
        Step();
        EXPECT_EQ(RegPair(pair), 0x97FF);

        // wraps around without setting any flags
        RegPair(pair) = 0x0000;
        Step();
        EXPECT_EQ(RegPair(pair), 0xFFFF);
        EXPECT_FLAGS(Flag::FIXED);
    }
}

struct IncDecCase
{
    uint8 value;
    uint8 result;
    uint8 flags; // without carry, which INR/DCR never change
};

class IncDecTest : public CPUTest
{
public:
    void Check(uint8 opcodeBase, std::initializer_list<IncDecCase> cases)
    {
        for (int reg = 0; reg < 8; reg++)
        {
            for (const IncDecCase& c : cases)
            {
                for (bool carry : {false, true})
                {
                    SCOPED_TRACE(testing::Message() << Reg::Name(reg) << " = 0x" << std::hex
                                 << int(c.value) << ", carry " << carry);
                    ResetAll();
                    cpu.HL = DATA;
                    Reg(reg) = c.value;
                    // set every other flag so stale values are caught
                    cpu.SetFlags(Flag::ALL & ~(carry ? 0 : Flag::CY));
                    Load({uint8(opcodeBase | reg << 3)});
                    Step();

                    EXPECT_EQ(Reg(reg), c.result);
                    EXPECT_FLAGS(c.flags | (carry ? Flag::CY : 0));
                }
            }
        }
    }
};

TEST_F(IncDecTest, INR_AllRegisters)
{
    Check(0x04, {
        {0x00, 0x01, Flag::FIXED},
        {0x0F, 0x10, Flag::AC | Flag::FIXED},
        {0x7F, 0x80, Flag::S | Flag::AC | Flag::FIXED},
        {0xFF, 0x00, Flag::Z | Flag::AC | Flag::P | Flag::FIXED},
        {0x02, 0x03, Flag::P | Flag::FIXED},
    });
}

TEST_F(IncDecTest, DCR_AllRegisters)
{
    // AC is set when the low nibble did not borrow
    Check(0x05, {
        {0x01, 0x00, Flag::Z | Flag::AC | Flag::P | Flag::FIXED},
        {0x00, 0xFF, Flag::S | Flag::P | Flag::FIXED},
        {0x10, 0x0F, Flag::P | Flag::FIXED},
        {0x80, 0x7F, Flag::FIXED},
        {0x42, 0x41, Flag::AC | Flag::P | Flag::FIXED},
    });
}

// =============================================================================
// arithmetic and logic
// =============================================================================

// ALU operations in opcode encoding order (10OOOSSS, 11OOO110 for immediates)
enum AluOp { ADD, ADC, SUB, SBB, ANA, XRA, ORA, CMP };
static const char* aluNames[] = {"ADD", "ADC", "SUB", "SBB", "ANA", "XRA", "ORA", "CMP"};

struct AluCase
{
    uint8 a;
    uint8 operand;
    bool carry;
    uint8 result;
    uint8 flags;
};

enum class Operand { Register, Memory, Immediate };

class AluTest : public CPUTest
{
public:
    struct Result { uint8 a; uint8 flags; };

    Result Execute(AluOp alu, Operand form, uint8 a, uint8 operand, uint8 flags, int reg = Reg::B)
    {
        ResetAll();
        cpu.HL = DATA;
        cpu.A = a;
        cpu.SetFlags(flags);

        switch (form)
        {
            case Operand::Register:
                if (reg != Reg::A)
                    Reg(reg) = operand;
                Load({uint8(0x80 | alu << 3 | reg)});
                break;
            case Operand::Memory:
                mem.Write(DATA, operand);
                Load({uint8(0x86 | alu << 3)});
                break;
            case Operand::Immediate:
                Load({uint8(0xC6 | alu << 3), operand});
                break;
        }

        Step();
        EXPECT_EQ(cpu.PC, ORIGIN + (form == Operand::Immediate ? 2 : 1));
        return {cpu.A, cpu.GetFlags()};
    }

    // runs every case through the register, memory and immediate form of the opcode
    void Check(AluOp alu, std::initializer_list<AluCase> cases)
    {
        const char* formNames[] = {"register", "memory", "immediate"};
        for (const AluCase& c : cases)
        {
            for (Operand form : {Operand::Register, Operand::Memory, Operand::Immediate})
            {
                // once with every other flag set and once with them clear, to catch stale flags
                for (uint8 otherFlags : {uint8(Flag::FIXED), uint8(Flag::ALL & ~Flag::CY)})
                {
                    SCOPED_TRACE(testing::Message() << aluNames[alu] << " (" << formNames[int(form)]
                                 << "): A=0x" << std::hex << int(c.a) << " operand=0x" << int(c.operand)
                                 << " carry=" << c.carry << " flags before=0x" << int(otherFlags));

                    Result r = Execute(alu, form, c.a, c.operand, otherFlags | (c.carry ? Flag::CY : 0));
                    EXPECT_EQ(r.a, c.result);
                    EXPECT_EQ(FlagString(r.flags), FlagString(c.flags));
                }
            }
        }
    }
};

TEST_F(AluTest, ADD)
{
    Check(ADD, {
        {0x00, 0x00, false, 0x00, Flag::Z | Flag::P | Flag::FIXED},
        {0x2E, 0x74, false, 0xA2, Flag::S | Flag::AC | Flag::FIXED},
        {0xFF, 0x01, false, 0x00, Flag::Z | Flag::AC | Flag::P | Flag::CY | Flag::FIXED},
        {0x80, 0x80, false, 0x00, Flag::Z | Flag::P | Flag::CY | Flag::FIXED},
        {0x0F, 0x01, false, 0x10, Flag::AC | Flag::FIXED},
        {0x01, 0x01, true,  0x02, Flag::FIXED}, // carry in is ignored
    });
}

TEST_F(AluTest, ADC)
{
    Check(ADC, {
        {0x01, 0x01, true,  0x03, Flag::P | Flag::FIXED},
        {0xFF, 0x00, true,  0x00, Flag::Z | Flag::AC | Flag::P | Flag::CY | Flag::FIXED},
        {0x0E, 0x01, true,  0x10, Flag::AC | Flag::FIXED},
        {0x3D, 0x42, false, 0x7F, Flag::FIXED},
        {0x7F, 0x00, true,  0x80, Flag::S | Flag::AC | Flag::FIXED},
        {0x80, 0x80, true,  0x01, Flag::CY | Flag::FIXED},
    });
}

// subtraction is done as A + ~operand + 1 on the 8080, AC is the carry out of bit 3 of that addition
TEST_F(AluTest, SUB)
{
    Check(SUB, {
        {0x3E, 0x3E, false, 0x00, Flag::Z | Flag::AC | Flag::P | Flag::FIXED},
        {0x00, 0x01, false, 0xFF, Flag::S | Flag::P | Flag::CY | Flag::FIXED},
        {0x10, 0x01, false, 0x0F, Flag::P | Flag::FIXED},
        {0x05, 0x02, false, 0x03, Flag::AC | Flag::P | Flag::FIXED},
        {0x80, 0x01, false, 0x7F, Flag::FIXED},
        {0x05, 0x02, true,  0x03, Flag::AC | Flag::P | Flag::FIXED}, // carry in is ignored
    });
}

TEST_F(AluTest, SBB)
{
    Check(SBB, {
        {0x04, 0x02, true,  0x01, Flag::AC | Flag::FIXED},
        {0x00, 0x00, true,  0xFF, Flag::S | Flag::P | Flag::CY | Flag::FIXED},
        {0x05, 0x05, false, 0x00, Flag::Z | Flag::AC | Flag::P | Flag::FIXED},
        {0x10, 0x00, true,  0x0F, Flag::P | Flag::FIXED},
        {0x00, 0xFF, true,  0x00, Flag::Z | Flag::P | Flag::CY | Flag::FIXED},
    });
}

// AND sets AC to the OR of bit 3 of both operands, carry is always cleared
TEST_F(AluTest, ANA)
{
    Check(ANA, {
        {0xFC, 0x0F, false, 0x0C, Flag::AC | Flag::P | Flag::FIXED},
        {0xF0, 0x0F, false, 0x00, Flag::Z | Flag::AC | Flag::P | Flag::FIXED},
        {0x81, 0x81, true,  0x81, Flag::S | Flag::P | Flag::FIXED},
        {0x30, 0x10, true,  0x10, Flag::FIXED},
    });
}

TEST_F(AluTest, XRA)
{
    Check(XRA, {
        {0x5C, 0x78, true,  0x24, Flag::P | Flag::FIXED},
        {0xAA, 0xAA, false, 0x00, Flag::Z | Flag::P | Flag::FIXED},
        {0x0F, 0xF0, false, 0xFF, Flag::S | Flag::P | Flag::FIXED},
        {0x01, 0x00, true,  0x01, Flag::FIXED},
    });
}

TEST_F(AluTest, ORA)
{
    Check(ORA, {
        {0x33, 0x0F, true,  0x3F, Flag::P | Flag::FIXED},
        {0x00, 0x00, false, 0x00, Flag::Z | Flag::P | Flag::FIXED},
        {0x80, 0x01, false, 0x81, Flag::S | Flag::P | Flag::FIXED},
        {0x40, 0x00, true,  0x40, Flag::FIXED},
    });
}

// compare sets the flags like SUB but leaves A untouched
TEST_F(AluTest, CMP)
{
    Check(CMP, {
        {0x0A, 0x05, false, 0x0A, Flag::AC | Flag::P | Flag::FIXED},
        {0x02, 0x05, false, 0x02, Flag::S | Flag::CY | Flag::FIXED},
        {0xE5, 0xE5, true,  0xE5, Flag::Z | Flag::AC | Flag::P | Flag::FIXED},
        {0x80, 0x01, false, 0x80, Flag::FIXED},
    });
}

// every source register must decode to the same operation as the immediate form
TEST_F(AluTest, AllSourceRegistersDecode)
{
    for (int alu = ADD; alu <= CMP; alu++)
    {
        for (int reg = 0; reg < 8; reg++)
        {
            for (uint8 carry : {uint8(0), uint8(Flag::CY)})
            {
                SCOPED_TRACE(testing::Message() << aluNames[alu] << " " << Reg::Name(reg) << " carry " << int(carry));
                const uint8 a = 0x5A;
                const uint8 values[] = {0x01, 0x23, 0x45, 0x67, uint8(DATA >> 8), uint8(DATA & 0xFF), 0x89, a};

                Result expected = Execute(AluOp(alu), Operand::Immediate, a, values[reg], Flag::FIXED | carry);
                ResetAll();
                cpu.BC = 0x0123;
                cpu.DE = 0x4567;
                cpu.HL = DATA;
                mem.Write(DATA, 0x89);
                cpu.A = a;
                cpu.SetFlags(Flag::FIXED | carry);
                Load({uint8(0x80 | alu << 3 | reg)});
                Step();

                EXPECT_EQ(cpu.A, expected.a);
                EXPECT_FLAGS(expected.flags);
            }
        }
    }
}

TEST_F(CPUTest, ADD_M_And_SUB_M_ReadThroughHL)
{
    Load({op::ADD_M, op::SUB_M});
    cpu.HL = 0x3456;
    mem.Write(0x3456, 0x10);
    cpu.A = 0x05;
    Step();
    EXPECT_EQ(cpu.A, 0x15);
    Step();
    EXPECT_EQ(cpu.A, 0x05);
}

TEST_F(CPUTest, DAD_AllPairs)
{
    for (int pair = 0; pair < 4; pair++)
    {
        SCOPED_TRACE(testing::Message() << "pair " << pair);
        ResetAll();
        Load({uint8(0x09 | pair << 4)});
        cpu.HL = 0xA17B;
        if (pair != Pair::HL)
            RegPair(pair) = 0x339F;
        cpu.SetFlags(Flag::ALL);
        Step();

        if (pair == Pair::HL)
        {
            EXPECT_EQ(cpu.HL, 0x42F6);
            EXPECT_FLAGS(Flag::ALL);
        }
        else
        {
            EXPECT_EQ(cpu.HL, 0xD51A);
            // only the carry flag is affected
            EXPECT_FLAGS(Flag::ALL & ~Flag::CY);
        }
    }
}

TEST_F(CPUTest, DAD_SetsCarryOnOverflow)
{
    Load({op::DAD_D});
    cpu.HL = 0xFFFF;
    cpu.DE = 0x0002;
    cpu.SetFlags(Flag::FIXED);
    Step();
    EXPECT_EQ(cpu.HL, 0x0001);
    EXPECT_FLAGS(Flag::CY | Flag::FIXED);
}

struct DaaCase
{
    uint8 a;
    uint8 flagsBefore;
    uint8 result;
    uint8 flags;
};

TEST_F(CPUTest, DAA)
{
    const DaaCase cases[] = {
        {0x9B, Flag::FIXED,            0x01, Flag::AC | Flag::CY | Flag::FIXED},
        {0x7D, Flag::FIXED,            0x83, Flag::S | Flag::AC | Flag::FIXED},
        {0x9A, Flag::FIXED,            0x00, Flag::Z | Flag::AC | Flag::P | Flag::CY | Flag::FIXED},
        {0x11, Flag::AC | Flag::FIXED, 0x17, Flag::P | Flag::FIXED},
        {0x20, Flag::CY | Flag::FIXED, 0x80, Flag::S | Flag::CY | Flag::FIXED},
        {0x00, Flag::CY | Flag::FIXED, 0x60, Flag::P | Flag::CY | Flag::FIXED},
        {0x42, Flag::FIXED,            0x42, Flag::P | Flag::FIXED},
    };

    for (const DaaCase& c : cases)
    {
        SCOPED_TRACE(testing::Message() << "A=0x" << std::hex << int(c.a) << " flags=" << FlagString(c.flagsBefore));
        ResetAll();
        Load({op::DAA});
        cpu.A = c.a;
        cpu.SetFlags(c.flagsBefore);
        Step();
        EXPECT_EQ(cpu.A, c.result);
        EXPECT_FLAGS(c.flags);
    }
}

TEST_F(CPUTest, DAA_AfterDecimalAddition)
{
    // 38 + 45 = 83 in BCD
    Load({op::MVI_A, 0x38, op::ADI, 0x45, op::DAA});
    Run(3);
    EXPECT_EQ(cpu.A, 0x83);
    EXPECT_FALSE(cpu.GetFlags() & Flag::CY);

    // 99 + 01 = 100 in BCD
    Load({op::MVI_A, 0x99, op::ADI, 0x01, op::DAA});
    Run(3);
    EXPECT_EQ(cpu.A, 0x00);
    EXPECT_TRUE(cpu.GetFlags() & Flag::CY);
}

TEST_F(CPUTest, CMA_ComplementsAWithoutFlags)
{
    Load({op::CMA});
    cpu.A = 0x51;
    cpu.SetFlags(Flag::FIXED);
    Step();
    EXPECT_EQ(cpu.A, 0xAE);
    EXPECT_FLAGS(Flag::FIXED);
}

TEST_F(CPUTest, STC_And_CMC)
{
    Load({op::STC, op::CMC, op::CMC});
    cpu.SetFlags(Flag::S | Flag::Z | Flag::AC | Flag::P | Flag::FIXED);
    Step();
    EXPECT_FLAGS(Flag::ALL);
    Step();
    EXPECT_FLAGS(Flag::ALL & ~Flag::CY);
    Step();
    EXPECT_FLAGS(Flag::ALL);
}

// =============================================================================
// rotates, only the carry flag is affected
// =============================================================================

struct RotateCase
{
    uint8 opcode;
    uint8 a;
    bool carry;
    uint8 result;
    bool carryAfter;
};

TEST_F(CPUTest, Rotates)
{
    const RotateCase cases[] = {
        {op::RLC, 0xF2, false, 0xE5, true},
        {op::RLC, 0x01, true,  0x02, false},
        {op::RRC, 0xF2, true,  0x79, false},
        {op::RRC, 0x01, false, 0x80, true},
        {op::RAL, 0xB5, false, 0x6A, true},
        {op::RAL, 0x00, true,  0x01, false},
        {op::RAL, 0x01, false, 0x02, false},
        {op::RAL, 0x80, false, 0x00, true},
        {op::RAR, 0x6A, true,  0xB5, false},
        {op::RAR, 0x01, false, 0x00, true},
    };

    for (const RotateCase& c : cases)
    {
        for (uint8 otherFlags : {uint8(Flag::FIXED), uint8(Flag::ALL & ~Flag::CY)})
        {
            SCOPED_TRACE(testing::Message() << "opcode 0x" << std::hex << int(c.opcode) << " A=0x" << int(c.a)
                         << " carry=" << c.carry << " flags=0x" << int(otherFlags));
            ResetAll();
            Load({c.opcode});
            cpu.A = c.a;
            cpu.SetFlags(otherFlags | (c.carry ? Flag::CY : 0));
            Step();
            EXPECT_EQ(cpu.A, c.result);
            EXPECT_FLAGS(otherFlags | (c.carryAfter ? Flag::CY : 0));
        }
    }
}

// =============================================================================
// branching
// =============================================================================

// condition encoding order (11CCC...): NZ, Z, NC, C, PO, PE, P, M
struct Condition
{
    const char* name;
    uint8 flag;
    bool takenWhenSet;
};

static const Condition conditions[8] = {
    {"NZ", Flag::Z,  false},
    {"Z",  Flag::Z,  true},
    {"NC", Flag::CY, false},
    {"C",  Flag::CY, true},
    {"PO", Flag::P,  false},
    {"PE", Flag::P,  true},
    {"P",  Flag::S,  false},
    {"M",  Flag::S,  true},
};

TEST_F(CPUTest, JMP)
{
    Load({op::JMP, 0x00, 0x3E});
    EXPECT_EQ(Step(), 10);
    EXPECT_EQ(cpu.PC, 0x3E00);
}

TEST_F(CPUTest, ConditionalJumps)
{
    for (int cc = 0; cc < 8; cc++)
    {
        for (bool flagSet : {false, true})
        {
            const Condition& cond = conditions[cc];
            bool taken = flagSet == cond.takenWhenSet;
            SCOPED_TRACE(testing::Message() << "J" << cond.name << ", flag set " << flagSet);

            ResetAll();
            cpu.SetFlags(Flag::FIXED | (flagSet ? cond.flag : 0));
            Load({uint8(0xC2 | cc << 3), 0x00, 0x30});

            EXPECT_EQ(Step(), 10);
            EXPECT_EQ(cpu.PC, taken ? 0x3000 : ORIGIN + 3);
        }
    }
}

TEST_F(CPUTest, CALL_And_RET)
{
    Load({op::CALL, 0x10, 0x01, op::HLT});
    Poke(0x0110, {op::MVI_A, 0x42, op::RET});
    cpu.SP = 0x4000;

    EXPECT_EQ(Step(), 17);
    EXPECT_EQ(cpu.PC, 0x0110);
    EXPECT_EQ(cpu.SP, 0x3FFE);
    EXPECT_EQ(PeekWord(0x3FFE), ORIGIN + 3);

    Step();
    EXPECT_EQ(Step(), 10);
    EXPECT_EQ(cpu.PC, ORIGIN + 3);
    EXPECT_EQ(cpu.SP, 0x4000);
    EXPECT_EQ(cpu.A, 0x42);
}

TEST_F(CPUTest, ConditionalCalls)
{
    for (int cc = 0; cc < 8; cc++)
    {
        for (bool flagSet : {false, true})
        {
            const Condition& cond = conditions[cc];
            bool taken = flagSet == cond.takenWhenSet;
            SCOPED_TRACE(testing::Message() << "C" << cond.name << ", flag set " << flagSet);

            ResetAll();
            cpu.SP = 0x4000;
            cpu.SetFlags(Flag::FIXED | (flagSet ? cond.flag : 0));
            Load({uint8(0xC4 | cc << 3), 0x00, 0x30});

            if (taken)
            {
                EXPECT_EQ(Step(), 17);
                EXPECT_EQ(cpu.PC, 0x3000);
                EXPECT_EQ(cpu.SP, 0x3FFE);
                EXPECT_EQ(PeekWord(0x3FFE), ORIGIN + 3);
            }
            else
            {
                EXPECT_EQ(Step(), 11);
                EXPECT_EQ(cpu.PC, ORIGIN + 3);
                EXPECT_EQ(cpu.SP, 0x4000);
            }
        }
    }
}

TEST_F(CPUTest, ConditionalReturns)
{
    for (int cc = 0; cc < 8; cc++)
    {
        for (bool flagSet : {false, true})
        {
            const Condition& cond = conditions[cc];
            bool taken = flagSet == cond.takenWhenSet;
            SCOPED_TRACE(testing::Message() << "R" << cond.name << ", flag set " << flagSet);

            ResetAll();
            cpu.SP = 0x3FFE;
            Poke(0x3FFE, {0x34, 0x12});
            cpu.SetFlags(Flag::FIXED | (flagSet ? cond.flag : 0));
            Load({uint8(0xC0 | cc << 3)});

            if (taken)
            {
                EXPECT_EQ(Step(), 11);
                EXPECT_EQ(cpu.PC, 0x1234);
                EXPECT_EQ(cpu.SP, 0x4000);
            }
            else
            {
                EXPECT_EQ(Step(), 5);
                EXPECT_EQ(cpu.PC, ORIGIN + 1);
                EXPECT_EQ(cpu.SP, 0x3FFE);
            }
        }
    }
}

TEST_F(CPUTest, RST_AllVectors)
{
    for (int n = 0; n < 8; n++)
    {
        SCOPED_TRACE(testing::Message() << "RST " << n);
        ResetAll();
        cpu.SP = 0x4000;
        Load({uint8(0xC7 | n << 3)});
        Step();
        EXPECT_EQ(cpu.PC, n * 8);
        EXPECT_EQ(cpu.SP, 0x3FFE);
        EXPECT_EQ(PeekWord(0x3FFE), ORIGIN + 1);
    }
}

TEST_F(CPUTest, PCHL_JumpsToHL)
{
    Load({op::PCHL});
    cpu.HL = 0x413E;
    Step();
    EXPECT_EQ(cpu.PC, 0x413E);
}
