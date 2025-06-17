#include <string>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
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

void cpu::clearAllFlags(){
    uint16_t AF = registers[0];
    uint8_t A = getUpper(AF);
    registers[0] = A << 8;
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

void cpu::inc8(uint8_t byte){
    // half carry
    if((byte & 0xF) + 1 > 0xF) {setFlag('H');} else {clearFlag('H');} 
    // dont touch carry flag

    if((uint8_t)(byte+1) == 0) {setFlag('Z');} else {clearFlag('Z');}
    clearFlag('N');
}

void cpu::dec8(uint8_t byte){
    // carry flags indicate borrows
    if((byte & 0xF) == 0) {setFlag('H');} else {clearFlag('H');} // if last nibble is zero, then it will borrow
    // dont touch carry flag

    if((uint8_t)(byte-1) == 0) {setFlag('Z');} else {clearFlag('Z');}
    setFlag('N');
}
/* no flags set when inc/dec 16 bit
void cpu::inc16(uint16_t bytes){
    
}


void cpu::dec16(uint16_t bytes){
    
}*/

void cpu::add16(uint16_t byte1, uint16_t byte2){
    clearFlag('N');
    if((byte1 & 0x0FFF) + (byte2 & 0x0FFF) > 0x0FFF){setFlag('H');} else {clearFlag('H');}
    if((byte1 + byte2) > 0xFFFF){setFlag('C');} else {clearFlag('C');}
}

// TODO: More helper funcs


int cpu::execute(){
    // switch based off of default table
    // return cycles
    uint8_t firstNibble = (opcode & 0xF0) >> 4;
    uint8_t secondNibble = opcode & 0x0F;
    switch(firstNibble){
        case(0x0):{
            switch (secondNibble)
            {
            case(0x0):
                // do nothing
                return 1;
                break;

            case(0x1):
                // LD BC, d16
                registers[1] = fetchNext2Bytes();
                return 3;
                break;
            
            case(0x2):
                MMU.writeMem(getUpper(registers[0]), registers[1]);
                return 2;
                break;
            
            case(0x3):
                registers[1]++;
                // no flag checks for 16 bit inc/dec
                return 2;
                break;
            
            case(0x4):
                uint8_t B = getUpper(registers[1]);
                uint8_t C = getLower(registers[1]);

                // flag check
                inc8(B);
                B++;

                registers[1] = (B << 8) | C;
                return 1;
                break;
        
            case(0x5):
                uint8_t B = getUpper(registers[1]);
                uint8_t C = getLower(registers[1]);

                // flag check
                dec8(B);
                B--;

                registers[1] = (B << 8) | C;
                return 1;
                break;
            
            case(0x6):
                uint8_t B = getUpper(registers[1]);
                uint8_t C = getLower(registers[1]);
                B = fetchNextByte();

                registers[1] = (B << 8) | C;
                return 2;
                break;
            
            case(0x7):
                uint8_t A = getUpper(registers[0]);
                uint8_t F = getLower(registers[0]);
                bool leftMost = A >> 7;
                if(leftMost){setFlag('C');} else {clearFlag('C');}
                A = (A << 1) | leftMost;
                registers[0] = (A << 8) | F;
                return 1;
                break;
            
            case(0x8):
                uint8_t lower = getLower(sp);
                uint8_t upper = getUpper(sp);
                uint16_t location = fetchNext2Bytes();
                MMU.writeMem(lower,location);
                MMU.writeMem(upper,location+1);
                return 5;
                break;
            
            case(0x9):
                uint16_t BC = registers[1];
                uint16_t HL = registers[3];
                registers[3] = BC + HL;

                // set flags
                add16(BC,HL);

                return 2;
                break;
            
            case(0xA):
                uint16_t BC = registers[1];
                uint8_t data = MMU.readMem(BC);
                uint8_t F = getLower(registers[0]);
                registers[0] = (data << 8) | F;

                return 2;
                break;
            
            case(0xB):
                registers[1]--;

                return 2;
                break;
            
            case(0xC):
                uint8_t B = getUpper(registers[1]);
                uint8_t C = getLower(registers[1]);

                // flag check
                inc8(C);
                C++;

                registers[1] = (B << 8) | C;
                
                return 1;
                break;
            
            case(0xD):
                uint8_t B = getUpper(registers[1]);
                uint8_t C = getLower(registers[1]);

                // flag check
                dec8(C);
                C--;

                registers[1] = (B << 8) | C;

                return 1;
                break;
            
            case(0xE):
                uint8_t B = getUpper(registers[1]);
                uint8_t C = getLower(registers[1]);
                C = fetchNextByte();

                registers[1] = (B << 8) | C;
                return 2;
                break;
        
            case(0xF):
                uint8_t A = getUpper(registers[0]);
                uint8_t F = getLower(registers[0]);
                bool rightMost = A & 0x1;
                if(rightMost){setFlag('C');} else {clearFlag('C');}
                A = (A >> 1) | (rightMost << 7);
                registers[0] = (A << 8) | F;

                return 1;
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x1):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO: do after figuring out interrupt flags*/ 
                break;
            
            case(0x1):
                registers[2] = fetchNext2Bytes();
                return 3;
                break;
            
            case(0x2):
                MMU.writeMem(getUpper(registers[0]), registers[2]);
                return 2;
                break;
            
            case(0x3):
                registers[2]++;
                // no flag checks for 16 bit inc/dec
                return 2;
                break;
            
            case(0x4):
                uint8_t D = getUpper(registers[2]);
                uint8_t E = getLower(registers[2]);

                // flag check
                inc8(D);
                D++;

                registers[2] = (D << 8) | E;
                return 1;
                break;
        
            case(0x5):
                uint8_t D = getUpper(registers[2]);
                uint8_t E = getLower(registers[2]);

                // flag check
                dec8(D);
                D--;

                registers[2] = (D << 8) | E;
                return 1;
                break;
            
            case(0x6):
                uint8_t D = getUpper(registers[1]);
                uint8_t E = getLower(registers[1]);
                D = fetchNextByte();

                registers[2] = (D << 8) | E;
                return 2;
                break;
            
            case(0x7):
                uint8_t A = getUpper(registers[0]);
                uint8_t F = getLower(registers[0]);
                bool leftMost = A >> 7;
                A = (A << 1) | (getFlag('C'));
                if(leftMost) {setFlag('C');} else {clearFlag('C');}
                return 1;
                break;
            
            case(0x8):
                uint8_t data = fetchNextByte();
                pc += data;
                return 3;
                break;
            
            case(0x9):
                uint16_t DE = registers[2];
                uint16_t HL = registers[3];
                registers[3] = DE + HL;

                // set flags
                add16(DE,HL);

                return 2;
                break;
            
            case(0xA):
                uint16_t DE = registers[2];
                uint8_t data = MMU.readMem(DE);
                uint8_t F = getLower(registers[0]);
                registers[0] = (data << 8) | F;

                return 2;
                break;
            
            case(0xB):
                registers[2]--;

                return 2;
                break;
            
            case(0xC):
                uint8_t D = getUpper(registers[2]);
                uint8_t E = getLower(registers[2]);

                // flag check
                inc8(E);
                E++;

                registers[1] = (D << 8) | E;
                
                return 1;
                break;
            
            case(0xD):
                uint8_t D = getUpper(registers[2]);
                uint8_t E = getLower(registers[2]);

                // flag check
                dec8(E);
                E--;

                registers[1] = (D << 8) | E;
                
                return 1;
                break;
            
            case(0xE):
                uint8_t D = getUpper(registers[2]);
                uint8_t E = getLower(registers[2]);
                E = fetchNextByte();

                registers[1] = (D << 8) | E;
                return 2;
                break;
                
            case(0xF):
                uint8_t A = getUpper(registers[0]);
                uint8_t F = getLower(registers[0]);
                bool rightMost = A & 0x1;
                if(rightMost){setFlag('C');} else {clearFlag('C');}
                A = (A >> 1) | (getFlag('C') << 7);
                return 1;
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x2):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x3):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x4):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x5):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x6):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x7):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x8):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x9):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xA):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }
        
        case(0xB):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xC):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xD):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xE):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xF):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }
        
        default: throw std::runtime_error("Invalid Opcode!"); break;
    }
}

int cpu::executePrefixed(){
    fetchOpcode();
    // switch based off of prefix table
    uint8_t firstNibble = (opcode & 0xF0) >> 4;
    uint8_t secondNibble = opcode & 0x0F;
    switch(firstNibble){
        case(0x0):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x1):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x2):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x3):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x4):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x5):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x6):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x7):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x8):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0x9):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xA):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }
        
        case(0xB):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xC):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xD):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xE):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }

        case(0xF):{
            switch (secondNibble)
            {
            case(0x0):
                /*TODO:*/
                break;
            
            case(0x1):
                /*TODO:*/
                break;
            
            case(0x2):
                /*TODO:*/
                break;
            
            case(0x3):
                /*TODO:*/
                break;
            
            case(0x4):
                /*TODO:*/
                break;
        
            case(0x5):
                /*TODO:*/
                break;
            
            case(0x6):
                /*TODO:*/
                break;
            
            case(0x7):
                /*TODO:*/
                break;
            
            case(0x8):
                /*TODO:*/
                break;
            
            case(0x9):
                /*TODO:*/
                break;
            
            case(0xA):
                /*TODO:*/
                break;
            
            case(0xB):
                /*TODO:*/
                break;
            
            case(0xC):
                /*TODO:*/
                break;
            
            case(0xD):
                /*TODO:*/
                break;
            
            case(0xE):
                /*TODO:*/
                break;
                
            case(0xF):
                /*TODO:*/
                break;
            
            default:
                throw std::runtime_error("Invalid Opcode!"); break;
            }
        }
        
        default: throw std::runtime_error("Invalid Opcode!"); break;
    }
}

int cpu::executeOpcode(){
    if(opcode == 0xCB){
        return executePrefixed();
    } else{
        return execute();
    }
}

#pragma endregion

// Constructor
cpu::cpu(mmu& MMUref, ppu& PPUref, timer& TIMERref): MMU(MMUref), PPU(PPUref), TIMER(TIMERref){
    memset(registers,0,sizeof(registers));
    pc = 0x0100; // Right above boot rom
    sp = 0xFFFE; // only lives in high ram for boot, then moves to work ram, downwardly    
    opcode = 0;
}

// Game loop
int cpu::step(){
    fetchOpcode();
    // return cycles to help sync with ppu and timer
}
