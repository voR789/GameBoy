#include "mmu.h"
#include <cstdint>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
// Timer registers are at:
#define REG_DIV  0xFF04
#define REG_TIMA 0xFF05
#define REG_TMA  0xFF06
#define REG_TAC  0xFF07

mmu::mmu(): divCounter(0){
    clearMem();
}

void mmu::clearMem(){
    memset(memory, 0, sizeof(memory));
}

// DIV logic
void mmu::addDivCounter(int cycles){
    divCounter += cycles;
}

uint16_t mmu::getDivCounter(){
    return divCounter;
}

uint8_t mmu::readMem(int index){
    // I/O Registers
    if (index >= 0xFF00 && index <= 0xFF7F) {
        switch (index) {
            case 0xFF00: return 0xCF;        // JOYP (no buttons pressed)
            case 0xFF01: return memory[index]; // SB
            case 0xFF02: return memory[index]; // SC
            case 0xFF04: return divCounter >> 8; // DIV
            case 0xFF05: return memory[index]; // TIMA
            case 0xFF06: return memory[index]; // TMA
            case 0xFF07: return memory[index]; // TAC
            case 0xFF0F: return memory[index]; // IF
            case 0xFF40: return 0x91; // LCDC
            case 0xFF41: return 0x85; // STAT
            case 0xFF44: return 0x00; // LY
            case 0xFF47: return 0xFC; // BGP
            default: return memory[index];
        }
    } 

    // Echo RAM
    if (index >= 0xE000 && index <= 0xFDFF) {
        return memory[index - 0x2000]; // Mirror of 0xC000–0xDDFF
    }

    return memory[index];    
}
void mmu::writeMem(uint8_t byte, int index) {
    
    if(index <=  0x7FFF){
        return;
    }
    // Echo RAM
    if (index >= 0xE000 && index <= 0xFDFF) {
        memory[index - 0x2000] = byte; // Write to mirror
        return;
    }

    // Unusable memory area
    if (index >= 0xFEA0 && index <= 0xFEFF) {
        return; // Not usable
    }

    // I/O Registers
    if (index >= 0xFF00 && index <= 0xFF7F) {
        switch (index) {
            case 0xFF01: // SB
                memory[index] = byte;
                break;
            case 0xFF02: // SC
                memory[index] = byte;
                if (byte == 0x81) {
                    std::cout << "" << static_cast<char>(memory[0xFF01]);
                    std::cout.flush();
                }
                break;
            case 0xFF04:
                divCounter = 0;
                break;
            case 0xFF07:
                memory[index] = byte;
                break;   
            case 0xFF0F:
                memory[index] = byte;
                // std::cout << "[MMU] IF written: 0x" << std::hex << (int)byte << "\n"; 
                break;
            default:
                memory[index] = byte;
                break;
        }
        return;
    }

    // --- Normal memory write ---
    memory[index] = byte;
}


void mmu::loadGame(const std::string& filename){
    std::ifstream rom(filename,std::ios::binary);
    std::vector<char> data((std::istreambuf_iterator<char>(rom)), std::istreambuf_iterator<char>()); // cart  vector
    for(int i = 0x0; i < data.size() && i < 0x8000; i++){
        memory[i] = data[i];
    }
}