#include "mmu.h"
#include <cstdint>
#include <iostream>
#include <fstream>
#include <vector>

uint8_t mmu::readMem(int index){
    if(index == 0xFF00)
    {
        return 0xFF;
    }
    if(index == 0xFF04)
    {
        return (fakeDIV++);
    } 
    return memory[index];
}

void mmu::writeMem(uint8_t byte, int index){
    memory[index] = byte;

    // serial output stub
    if (index == 0xFF02 && byte == 0x81) {
        std::cout << (char)memory[0xFF01];
        memory[0xFF02] = 0x00; // mark transfer complete
        return;
    }
    if (index == 0xFF04) {
    fakeDIV = 0; // writing to DIV resets it
    return;
    }
}

void mmu::loadGame(const std::string& filename){
    std::ifstream rom(filename,std::ios::binary);
    std::vector<char> data((std::istreambuf_iterator<char>(rom)), std::istreambuf_iterator<char>()); // cart  vector
    for(int i = 0; i < data.size(); i++){
        memory[i] = data[i];
    }
}