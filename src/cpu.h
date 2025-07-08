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
#define IE_REGISTER 0xFFFF // interrupt enable register
#define IF_REGISTER 0xFF0F // Interrupt flag
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
        uint64_t cycles;
        bool IME;
        bool stop;
        bool halt;
        int EI_COUNTER;
        bool preInterruptStatus;
        // TODO: implement timers interrupts, controls, and gfx later
        
    public:
        // constructor
        cpu(mmu& MMUref, ppu& PPUref, timer& TIMERref); // TODO: 
        // helper functions
        uint8_t getUpper(uint16_t data);
        uint8_t getLower(uint16_t data);
        bool getFlag(char flag);
        void setFlag(char flag);
        void clearFlag(char flag);
        void fetchOpcode();
        uint8_t fetchNextByte();
        uint16_t fetchNext2Bytes();
        void inc8(char reg);
        void dec8(char reg);
        uint16_t add16(uint16_t byte1, uint16_t byte2);
        void addA(uint8_t byte);
        void adcA(uint8_t byte);
        void subA(uint8_t byte);
        void sbcA(uint8_t byte);
        void andA(uint8_t byte);
        void xorA(uint8_t byte);
        void orA(uint8_t byte);
        void cpA(uint8_t byte);

        uint8_t popStack();
        void pushStack(uint8_t byte);
        void storePC();
        
        void ldMem8(int address, uint8_t byte);
        void ldReg8(char reg, uint8_t byte);
        
        // prefixed helpers
        uint8_t RLC(uint8_t byte); // rotate left, store bit in carry
        uint8_t RL(uint8_t byte); // rotate left, through carry
        uint8_t RRC(uint8_t byte); // rotate right, store bit in carry
        uint8_t RR(uint8_t byte); // rotate right, through carry 
        uint8_t SLA(uint8_t byte); // shift left, through carry flag, bit 0 = 0
        uint8_t SRA(uint8_t byte); // shift right, through carry flag, bit 7 unchanged
        uint8_t SWAP(uint8_t byte); // swap nibbles
        uint8_t SRL(uint8_t byte); // shift right, through carry flag, bit 7 = 0
        void BIT_(uint8_t byte, int bit); // copy ! bit _ into Z flag
        uint8_t RES_(uint8_t byte, int bit); // reset bit _ 
        uint8_t SET_(uint8_t byte, int bit); // set bit _
        
        // hardware functions
        int step();
        int execute();
        int executePrefixed();
        int executeOpcode();
        int handleInterrupts();
        void triggerVBLankInterrupt();
        
};


#endif