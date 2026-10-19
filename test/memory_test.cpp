#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "memory.h"

TEST(MemoryTest, StartsCleared)
{
    auto mem = std::make_unique<Memory>();
    for (uint32 address = 0; address < MEMORY_SIZE; address++)
        ASSERT_EQ(mem->Read(address), 0) << "at address " << address;
}

TEST(MemoryTest, WriteThenRead)
{
    auto mem = std::make_unique<Memory>();
    mem->Write(0x1234, 0xAB);
    EXPECT_EQ(mem->Read(0x1234), 0xAB);
    EXPECT_EQ(mem->Read(0x1233), 0x00);
    EXPECT_EQ(mem->Read(0x1235), 0x00);
}

TEST(MemoryTest, CoversTheFull16BitAddressRange)
{
    auto mem = std::make_unique<Memory>();
    mem->Write(0x0000, 0x11);
    mem->Write(0xFFFF, 0x22);
    EXPECT_EQ(mem->Read(0x0000), 0x11);
    EXPECT_EQ(mem->Read(0xFFFF), 0x22);
}

TEST(MemoryTest, ConstructorCopiesInitialData)
{
    std::vector<uint8> init(MEMORY_SIZE);
    for (uint32 i = 0; i < MEMORY_SIZE; i++)
        init[i] = i & 0xFF;

    auto mem = std::make_unique<Memory>(init.data());
    for (uint32 address = 0; address < MEMORY_SIZE; address++)
        ASSERT_EQ(mem->Read(address), address & 0xFF) << "at address " << address;
}

TEST(MemoryTest, ClearZeroesEverything)
{
    auto mem = std::make_unique<Memory>();
    mem->Write(0x0000, 0x01);
    mem->Write(0x8000, 0x02);
    mem->Write(0xFFFF, 0x03);
    mem->Clear();
    EXPECT_EQ(mem->Read(0x0000), 0);
    EXPECT_EQ(mem->Read(0x8000), 0);
    EXPECT_EQ(mem->Read(0xFFFF), 0);
}

TEST(MemoryTest, RawSharesTheSameStorage)
{
    auto mem = std::make_unique<Memory>();
    mem->Raw()[0x4000] = 0x5A;
    EXPECT_EQ(mem->Read(0x4000), 0x5A);

    mem->Write(0x4001, 0xA5);
    EXPECT_EQ(mem->Raw()[0x4001], 0xA5);
}
