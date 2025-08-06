#ifndef TIMER_H
#define TIMER_H
#include <cstdint>
#include <iostream>
#include "mmu.h"

class timer{
    private:
        mmu& MMU;
        uint16_t divCounter; // Represents DIV
        uint8_t TIMA;
        uint8_t TMA;
        uint8_t TAC;
        int clockBit;
        bool overflowPending;
        int overflowDelay;

        // Debug
        int totalCycles;
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
            return TAC;
        }
        void writeTAC(uint8_t byte){
            // Watch for new falling edge
            // Store old info
            int oldClockBit = clockBit;
            uint8_t oldEnable = TAC & 0x4;
            uint8_t oldBit = (divCounter >> oldClockBit) & 0x1;
            
            // Change TAC
            TAC = (byte & 0x07); // Mask out unused bits
            clockBit = setClockBit();

            // Compare with new info
            uint8_t newEnable = TAC & 0x4;
            uint8_t newBit = (divCounter >> clockBit) & 0x1;
            
            if(oldEnable && newEnable){
                if(oldBit == 1 && newBit == 0){
                    if(TIMA == 0xFF){
                        // NO delay on TAC falling edge
                        TIMA = TMA;
                        setInterrupt();
                    } else{
                        TIMA++;
                    }
                }
            }
        }

        void tick();
        int setClockBit();
        void setInterrupt();
}; 

#endif