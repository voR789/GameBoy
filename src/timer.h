#ifndef TIMER_H
#define TIMER_H
#include <cstdint>
#include "mmu.h"

class timer{
    private:
        mmu& MMU;
        int totalCycles;
        uint16_t divCounter; // Represents DIV
        uint8_t TIMA;
        uint8_t TMA;
        uint8_t TAC;
        int clockBit;
    public:
        timer(mmu& mmu_ref);
        // getter and setter funcs
        uint8_t readDIV(){
            return divCounter >> 8;
        }
        void writeDIV(uint8_t byte){
            (void)byte;
            divCounter = 0;
        }
        uint8_t readTIMA(){
            return TIMA;
        }
        void writeTIMA(uint8_t byte){
            TIMA = byte;
        }
        uint8_t readTMA(){
            return TMA;
        }
        void writeTMA(uint8_t byte){
            TMA = byte;
        }
        uint8_t readTAC(){
            clockBit = setClockBit();
            return TAC;
        }
        void writeTAC(uint8_t byte){
            TAC = (byte & 0x07); // Mask out unused bits
        }

        void tick();
        int setClockBit();
        void setInterrupt();
}; 

#endif