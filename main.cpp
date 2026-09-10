#include "cpu.h"
#include "memory.h"

#include <iostream>
#include <vector>
#include <string.h>

bool loadFile(const char*, Memory* memory);
void loadCPM80(CPU* cpu, Memory* memory);
static void cpm80_port_out(uint8 port, uint8 value);

CPU* cpu = new CPU();
Memory* memory = new Memory();

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        fprintf(stderr, "error: no file was given'.\n");
        return 0;
    }

    // load the 'bios' 
    loadCPM80(cpu, memory);

    // load the given file
    if (!loadFile(argv[1], memory))
        return 0;

    while (!cpu->IsHalted())
    {
        cpu->Tick(memory);
    }

    delete cpu;
    delete memory;

    return 0;
}

bool loadFile(const char* filePath, Memory* memory)
{
    // load a file at 0x0100
    FILE* f = fopen(filePath, "rb");
    if (f == NULL) {
        fprintf(stderr, "error: can't open file '%s'.\n", filePath);
        return false;
    }

    // file size check:
    fseek(f, 0, SEEK_END);
    size_t fileSize = ftell(f);
    rewind(f);

    // in cpm80 systems the origin is at 0x0100
    static uint16 startAddress = 0x0100;

    if (startAddress + 0x0100 >= MEMORY_SIZE) {
        fprintf(stderr, "error: file %s can't fit in memory.\n", filePath);
        return false;
    }

    // read the file into a memory buffer
    uint8 initMemory[MEMORY_SIZE];
    memset(initMemory, 0, MEMORY_SIZE);
    size_t result = fread(&initMemory[startAddress], sizeof(uint8), fileSize, f);
    if (result != fileSize) {
        fprintf(stderr, "error: while reading file '%s'\n", filePath);
        return false;
    }

    // copy the loaded file into memory
    uint8* rawMem = memory->Raw();
    memcpy(&rawMem[startAddress], &initMemory[startAddress], fileSize);

    fclose(f);
    return true;
}

// loads the CP/M-80 BIOS
void loadCPM80(CPU* cpu, Memory* memory)
{
    // WBOOT - JMP WBOOT - Jump to 0x0100
    memory->Write(0x0000, 0xD3); // OUT
    memory->Write(0x0001, 0x00); // out port
    memory->Write(0x0002, 0xC9); // RET

    // IOBYTE
    memory->Write(0x0003, 0x00); // null

    // Drive Byte (null)
    memory->Write(0x0004, 0x00); // null

    // BDOS - out A to port 1
    memory->Write(0x0005, 0xD3); // OUT
    memory->Write(0x0006, 0x00); // out port
    memory->Write(0x0007, 0xC9); // RET

    // set the cpm80 bios call handler
    cpu->SetPortOutHandler(cpm80_port_out);
}

void cpm80_port_out(uint8 port, uint8 value)
{
    uint16 address = cpu->PC;
    switch (address)
    {
        case 0x0002: // WBOOT
        {
            static bool initialBoot = true;
            if (initialBoot)
            {
                initialBoot = false;
                cpu->PC = 0x0100;                         // start at 0x0100
            }                
            else
            {
                printf("\n"); // print a newline as the cpu halted.
                cpu->Halt();
            }
  
            break;
        }
        case 0x0007: // BDOS
        {
            uint8 operation = cpu->C;

            if (operation == 2)
            {
                // print a character stored in E
                printf("%c", cpu->E);
            }
            else if (operation == 9)
            {
                // print from memory at (DE) until '$' char
                uint16 addr = (cpu->D << 8) | cpu->E;
                do
                {
                    printf("%c", memory->Read(addr++));
                } while (memory->Read(addr) != '$');
            }
            break;
        }
    }
}
