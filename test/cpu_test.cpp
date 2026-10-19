#include "test_helpers.h"

// --- state --------------------------------------------------------------------

TEST_F(CPUTest, StartsCleared)
{
    EXPECT_EQ(cpu.A, 0);
    EXPECT_EQ(cpu.BC, 0);
    EXPECT_EQ(cpu.DE, 0);
    EXPECT_EQ(cpu.HL, 0);
    EXPECT_EQ(cpu.PC, 0);
    EXPECT_EQ(cpu.SP, 0);
    EXPECT_FLAGS(Flag::FIXED);
    EXPECT_FALSE(cpu.IsHalted());
    EXPECT_FALSE(cpu.IsInterruptEnabled());
}

TEST_F(CPUTest, RegisterPairsAliasTheirRegisters)
{
    // the first register of a pair is the high byte
    cpu.BC = 0x1234;
    cpu.DE = 0x5678;
    cpu.HL = 0x9ABC;
    EXPECT_EQ(cpu.B, 0x12);
    EXPECT_EQ(cpu.C, 0x34);
    EXPECT_EQ(cpu.D, 0x56);
    EXPECT_EQ(cpu.E, 0x78);
    EXPECT_EQ(cpu.H, 0x9A);
    EXPECT_EQ(cpu.L, 0xBC);

    cpu.B = 0xAA;
    cpu.E = 0xBB;
    cpu.L = 0xCC;
    EXPECT_EQ(cpu.BC, 0xAA34);
    EXPECT_EQ(cpu.DE, 0x56BB);
    EXPECT_EQ(cpu.HL, 0x9ACC);
}

TEST_F(CPUTest, ClearResetsAllState)
{
    cpu.A = 1; cpu.BC = 2; cpu.DE = 3; cpu.HL = 4; cpu.PC = 5; cpu.SP = 6;
    cpu.SetFlags(Flag::ALL);
    Load({op::EI, op::HLT});
    Run(2);
    ASSERT_TRUE(cpu.IsHalted());

    cpu.Clear();
    EXPECT_EQ(cpu.A, 0);
    EXPECT_EQ(cpu.BC, 0);
    EXPECT_EQ(cpu.DE, 0);
    EXPECT_EQ(cpu.HL, 0);
    EXPECT_EQ(cpu.PC, 0);
    EXPECT_EQ(cpu.SP, 0);
    EXPECT_FLAGS(Flag::FIXED);
    EXPECT_FALSE(cpu.IsHalted());
    EXPECT_FALSE(cpu.IsInterruptEnabled());
}

TEST_F(CPUTest, ResetOnlyRestartsAtZero)
{
    Load({op::MVI_A, 0x42, op::HLT});
    cpu.BC = 0x1234;
    Run(2);
    ASSERT_TRUE(cpu.IsHalted());

    cpu.Reset();
    EXPECT_EQ(cpu.PC, 0x0000);
    EXPECT_FALSE(cpu.IsHalted());
    EXPECT_EQ(cpu.A, 0x42);
    EXPECT_EQ(cpu.BC, 0x1234);
}

// --- timing -------------------------------------------------------------------

TEST_F(CPUTest, TickSpreadsAnOpcodeOverItsCycles)
{
    Load({op::MVI_A, 0x11, op::MVI_B, 0x22});

    // the opcode executes on its first tick...
    cpu.Tick(&mem);
    EXPECT_EQ(cpu.A, 0x11);
    EXPECT_EQ(cpu.PC, ORIGIN + 2);

    // ...then the cpu is busy for the remaining 6 of its 7 cycles
    for (int i = 0; i < 6; i++)
        cpu.Tick(&mem);
    EXPECT_EQ(cpu.B, 0x00);
    EXPECT_EQ(cpu.PC, ORIGIN + 2);

    cpu.Tick(&mem);
    EXPECT_EQ(cpu.B, 0x22);
}

TEST_F(CPUTest, StepFinishesAnOpcodeInProgress)
{
    Load({op::MVI_A, 0x11, op::MVI_B, 0x22});
    cpu.Tick(&mem);

    EXPECT_EQ(Step(), 7);
    EXPECT_EQ(cpu.B, 0x22);
    EXPECT_EQ(cpu.PC, ORIGIN + 4);
}

// cycle counts from the Intel 8080 datasheet, with all flags cleared so that
// RNZ/RNC/RPO/RP and CNZ/CNC/CPO/CP are taken and the other conditionals are not
static const uint8 datasheetCycles[0x100] = {
//  0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
    4,  10, 7,  5,  5,  5,  7,  4,  4,  10, 7,  5,  5,  5,  7,  4,  // 0x00
    4,  10, 7,  5,  5,  5,  7,  4,  4,  10, 7,  5,  5,  5,  7,  4,  // 0x10
    4,  10, 16, 5,  5,  5,  7,  4,  4,  10, 16, 5,  5,  5,  7,  4,  // 0x20
    4,  10, 13, 5,  10, 10, 10, 4,  4,  10, 13, 5,  5,  5,  7,  4,  // 0x30
    5,  5,  5,  5,  5,  5,  7,  5,  5,  5,  5,  5,  5,  5,  7,  5,  // 0x40
    5,  5,  5,  5,  5,  5,  7,  5,  5,  5,  5,  5,  5,  5,  7,  5,  // 0x50
    5,  5,  5,  5,  5,  5,  7,  5,  5,  5,  5,  5,  5,  5,  7,  5,  // 0x60
    7,  7,  7,  7,  7,  7,  7,  7,  5,  5,  5,  5,  5,  5,  7,  5,  // 0x70
    4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,  // 0x80
    4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,  // 0x90
    4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,  // 0xA0
    4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,  // 0xB0
    11, 10, 10, 10, 17, 11, 7,  11, 5,  10, 10, 10, 11, 17, 7,  11, // 0xC0
    11, 10, 10, 10, 17, 11, 7,  11, 5,  10, 10, 10, 11, 17, 7,  11, // 0xD0
    11, 10, 10, 18, 17, 11, 7,  11, 5,  5,  10, 4,  11, 17, 7,  11, // 0xE0
    11, 10, 10, 4,  17, 11, 7,  11, 5,  5,  10, 4,  11, 17, 7,  11, // 0xF0
};

TEST_F(CPUTest, CycleCountsMatchTheDatasheet)
{
    for (int opcode = 0; opcode < 0x100; opcode++)
    {
        ResetAll();
        cpu.SP = 0x4000;
        Load({uint8(opcode), 0x00, 0x30});
        EXPECT_EQ(Step(), datasheetCycles[opcode]) << "opcode 0x" << std::hex << opcode;
    }
}

// --- halting ------------------------------------------------------------------

TEST_F(CPUTest, HltStopsExecution)
{
    Load({op::HLT, op::MVI_A, 0x42});
    Step();
    EXPECT_TRUE(cpu.IsHalted());
    EXPECT_EQ(cpu.PC, ORIGIN + 1);

    Run(10);
    EXPECT_EQ(cpu.PC, ORIGIN + 1);
    EXPECT_EQ(cpu.A, 0x00);
}

TEST_F(CPUTest, ExternalHaltStopsExecution)
{
    Load({op::MVI_A, 0x42});
    cpu.Halt();
    Run(10);
    EXPECT_EQ(cpu.PC, ORIGIN);
    EXPECT_EQ(cpu.A, 0x00);
}

// --- interrupts ---------------------------------------------------------------

TEST_F(CPUTest, EiAndDiToggleInterrupts)
{
    Load({op::EI, op::DI});
    Step();
    EXPECT_TRUE(cpu.IsInterruptEnabled());
    Step();
    EXPECT_FALSE(cpu.IsInterruptEnabled());
}

TEST_F(CPUTest, InterruptIsIgnoredWhenDisabled)
{
    Load({op::NOP});
    cpu.SP = 0x4000;
    cpu.Interrupt(op::RST1);
    Step();
    EXPECT_EQ(cpu.PC, ORIGIN + 1);
    EXPECT_EQ(cpu.SP, 0x4000);
}

TEST_F(CPUTest, InterruptExecutesTheInjectedOpcode)
{
    Load({op::EI, op::NOP});
    cpu.SP = 0x4000;
    Step();

    cpu.Interrupt(op::RST1);
    EXPECT_EQ(Step(), 11);

    // the address of the opcode that would have run next is pushed
    EXPECT_EQ(cpu.PC, 0x0008);
    EXPECT_EQ(cpu.SP, 0x3FFE);
    EXPECT_EQ(PeekWord(0x3FFE), ORIGIN + 1);

    // further interrupts are disabled until EI
    EXPECT_FALSE(cpu.IsInterruptEnabled());
}

TEST_F(CPUTest, InterruptIsIgnoredWhileAnotherIsPending)
{
    Load({op::EI, op::NOP});
    cpu.SP = 0x4000;
    Step();

    cpu.Interrupt(op::RST1);
    cpu.Interrupt(op::RST2);
    Step();
    EXPECT_EQ(cpu.PC, 0x0008);
}

TEST_F(CPUTest, InterruptWakesAHaltedCPU)
{
    Load({op::EI, op::HLT});
    cpu.SP = 0x4000;
    Run(2);
    ASSERT_TRUE(cpu.IsHalted());

    cpu.Interrupt(op::RST7);
    Step();
    EXPECT_FALSE(cpu.IsHalted());
    EXPECT_EQ(cpu.PC, 0x0038);
    EXPECT_EQ(PeekWord(0x3FFE), ORIGIN + 2);
}

// --- ports --------------------------------------------------------------------

TEST_F(CPUTest, OutWritesAToThePort)
{
    int calls = 0;
    uint8 port = 0, value = 0;
    cpu.SetPortOutHandler([&](uint8 p, uint8 v) { calls++; port = p; value = v; });

    Load({op::OUT, 0x42});
    cpu.A = 0x99;
    Step();

    EXPECT_EQ(calls, 1);
    EXPECT_EQ(port, 0x42);
    EXPECT_EQ(value, 0x99);
    EXPECT_EQ(cpu.PC, ORIGIN + 2);
}

TEST_F(CPUTest, InReadsThePortIntoA)
{
    uint8 port = 0;
    cpu.SetPortInHandler([&](uint8 p) { port = p; return uint8(0x5A); });

    Load({op::IN, 0x10});
    cpu.SetFlags(Flag::FIXED);
    Step();

    EXPECT_EQ(port, 0x10);
    EXPECT_EQ(cpu.A, 0x5A);
    EXPECT_EQ(cpu.PC, ORIGIN + 2);
    EXPECT_FLAGS(Flag::FIXED);
}

TEST_F(CPUTest, UnconnectedPortsReadZero)
{
    Load({op::OUT, 0x01, op::IN, 0x01});
    cpu.A = 0xFF;
    Run(2);
    EXPECT_EQ(cpu.A, 0x00);
}
