#ifndef CPU_H
#define CPU_H
#include <cstdint>
#include <string>
#include "mmu.h"
#include "ppu.h"
#include "timer.h"

#define Z_FLAG 0x80 // bit 7 of F 1000 0000 - Zero flag
#define N_FLAG 0x40 // bit 6 of F 0100 0000 - Subtract flag
#define H_FLAG 0x20 // bit 5 of F 0010 0000 - Half carry flag, overflow in lower nibble
#define C_FLAG 0x10 // bit 4 of F 0001 0000 - Full carry flag, overflow in higher nibble

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
        int cycles;
        // TODO: implement timers interrupts, controls, and gfx later
        
    public:
        // constructor
        cpu(mmu& MMUref, ppu& PPUref, timer& TIMERref); // TODO: 
        // helper functions
        uint8_t getUpper(uint16_t data);
        uint8_t getLower(uint16_t data);
        void clearAllFlags();
        bool getFlag(char flag);
        void setFlag(char flag);
        void clearFlag(char flag);
        void fetchOpcode();
        uint8_t fetchNextByte();
        uint16_t fetchNext2Bytes();
        void inc8(uint8_t byte);
        void dec8(uint8_t byte);

        // hardware functions
        int step();
        int execute();
        int executePrefixed();
        int executeOpcode();
        
};


#endif