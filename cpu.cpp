#include <string>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <GL/glut.h>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"
#include "timer.h"


using std::string, std::memset, std::ifstream, std::cerr;

// Helper functions
#pragma region 
uint8_t cpu::getUpper(uint16_t data){
    return ((data & 0xFF00) >> 8);
}

uint8_t cpu::getLower(uint16_t data){
    return (data & 0xFF);
}

bool cpu::getFlag(char flag){
    uint8_t F = getLower(registers[4]);
    switch (flag) {
    case('Z'): return (F & Z_FLAG) != 0;
    case('N'): return (F & Z_FLAG) != 0;
    case('H'): return (F & Z_FLAG) != 0;
    case('C'): return (F & Z_FLAG) != 0;
    default: return false;
    }
}

void cpu::setFlag(char flag){
    uint8_t F = getLower(registers[4]);
    switch (flag) {
    case('Z'): F |= Z_FLAG; break;
    case('N'): F |= N_FLAG; break;
    case('H'): F |= H_FLAG; break;
    case('C'): F |= C_FLAG; break;
    default: break;
    }
    // Pushing changes to F register
    registers[4] |= F;
}

void cpu::clearFlag(char flag){
uint8_t F = getLower(registers[4]);
    switch (flag) {
    case('Z'): F &= ~Z_FLAG; break;
    case('N'): F &= ~N_FLAG; break;
    case('H'): F &= ~H_FLAG; break;
    case('C'): F &= ~C_FLAG; break;
    default: break;
    }
    // Pushing changes to F register
    registers[4] &= F;
}

void cpu::fetchOpcode(){
    opcode = MMU.readMem(pc++);
}

uint8_t cpu::fetchNextByte(){
    return MMU.readMem(pc++);
}

uint16_t cpu::fetchNext2Bytes(){
    return ((MMU.readMem(pc++) << 8) | MMU.readMem(pc++));
}

void cpu::execute(){
    // switch based off of default table
    case():{
        switch (expression)
        {
        case constant expression:
            /* code */
            break;
        
        default:
            break;
        }
    }
}

void cpu::executePrefixed(){
    fetchOpcode();
    // switch based off of prefix table
}

void cpu::executeOpcode(){
    if(opcode == 0xCB){
        executePrefixed();
    } else{
        execute();
    }
}

#pragma endregion

// Constructor
cpu::cpu(mmu& MMU, ppu& PPU, timer& TIMER): MMU(MMU), PPU(PPU), TIMER(TIMER){
    memset(registers,0,sizeof(registers));
    pc = 0x0100; // Right above boot rom
    sp = 0xFFFE; // only lives in high ram for boot, then moves to work ram, downwardly    
    opcode = 0;
}

// Game loop
void cpu::step(){
    fetchOpcode();

}