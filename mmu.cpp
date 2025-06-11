#include "mmu.h"
#include <cstdint>

uint8_t mmu::readMem(int index){
    return memory[index];
}

void mmu::writeMem(uint8_t byte, int index){
    memory[index] = byte;
}