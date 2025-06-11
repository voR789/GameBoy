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

void cpu::executePrefixed(){
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