#ifndef CPU_H
#define CPU_H
#include <cstdint>
#include <string>
#include "cpu.h"
#include "ppu.h"
#include "timer.h"

#define Z_FLAG 0x80 // bit 7 of F 1000 0000
#define N_FLAG 0x40 // bit 6 of F 0100 0000
#define H_FLAG 0x20 // bit 5 of F 0010 0000
#define C_FLAG 0x10 // bit 4 of F 0001 0000

class cpu{
    private:
        mmu& MMU;
        ppu& PPU;
        timer& TIMER;
        uint16_t registers[4];
        // AF, BC, DE, and HL - can be split into 8 bit registers
        // A - accumulator
        // F - Flags (7 = z) (6 = n) (5 = h) (4 = c)
        uint16_t sp;
        uint16_t pc; 
        uint16_t opcode;
        // TODO: implement timers interrupts, controls, and gfx later
        
    public:
        // constructor
        cpu(mmu& MMU, ppu& PPU, timer& TIMER); // TODO: 
        // helper functions
        uint8_t getUpper(uint16_t data);
        uint8_t getLower(uint16_t data);
        bool getFlag(char flag);
        void setFlag(char flag);
        void clearFlag(char flag);
        
        // hardware functions
        void step();
        
};


#endif