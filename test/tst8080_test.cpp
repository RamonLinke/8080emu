#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "cpu.h"
#include "memory.h"

namespace
{
    struct CPMResult
    {
        std::string output;
        bool finished; // returned to CP/M through WBOOT
    };

    // runs a CP/M .COM program with a minimal BDOS that only supports console output
    CPMResult RunCPMProgram(const char* path, uint32 maxOpcodes)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file)
        {
            ADD_FAILURE() << "can't open " << path;
            return {"", false};
        }
        std::vector<uint8> program((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        auto cpu = std::make_unique<CPU>();
        auto mem = std::make_unique<Memory>();

        // programs are loaded at 0x0100
        for (size_t i = 0; i < program.size(); i++)
            mem->Write(0x0100 + i, program[i]);

        // WBOOT halts, BDOS is handled below before its RET runs
        mem->Write(0x0000, CPUOpcode::HLT);
        mem->Write(0x0005, CPUOpcode::RET);
        cpu->PC = 0x0100;

        CPMResult result = {"", false};
        for (uint32 i = 0; i < maxOpcodes && !cpu->IsHalted(); i++)
        {
            if (cpu->PC == 0x0005)
            {
                if (cpu->C == 2)
                {
                    // print the character in E
                    result.output += char(cpu->E);
                }
                else if (cpu->C == 9)
                {
                    // print from (DE) until '$'
                    for (uint16 address = cpu->DE; mem->Read(address) != '$'; address++)
                        result.output += char(mem->Read(address));
                }
            }

            cpu->Step(mem.get());
        }

        result.finished = cpu->IsHalted() && cpu->PC == 0x0001;
        return result;
    }
}

// Microcosm Associates' 8080/8085 CPU diagnostic (test/TST8080.ASM)
TEST(TST8080, CPUIsOperational)
{
    CPMResult result = RunCPMProgram(TST8080_COM, 1000000);

    EXPECT_TRUE(result.finished) << "program did not return to CP/M, output:\n" << result.output;
    EXPECT_NE(result.output.find("CPU IS OPERATIONAL"), std::string::npos) << result.output;
    EXPECT_EQ(result.output.find("CPU HAS FAILED"), std::string::npos) << result.output;
}
