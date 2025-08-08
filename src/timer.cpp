#include "timer.h"

#include <bitset>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "mmu.h"

// Timer registers are at:
#define REG_DIV 0xFF04
#define REG_TIMA 0xFF05
#define REG_TMA 0xFF06
#define REG_TAC 0xFF07

timer::timer(mmu &MMU_REF) 
    : MMU(MMU_REF), divCounter(0), TIMA(0), TMA(0), TAC(0), clockBit(0), overflowPending(0), overflowDelay(0) {
    MMU.linkTIMER(this);
}

void timer::tick() {
    divCounter++;
    if(TAC & 0x4){ // bit 2 = enabled
        // falling edge on clockBit -> set on TAC write
        uint8_t oldBit = ((divCounter - 1) >> clockBit) & 0x1;
        uint8_t newBit = (divCounter >> clockBit) & 0x1;
        if(oldBit == 1 && newBit == 0){
            if(TIMA == 0xFF){
                overflowPending = true;
                overflowDelay = 4;
            } else{
                TIMA++;
            }
        }
    }

    // Handle delayed TIMA reload
    if (overflowPending) {
        overflowDelay--;
        if (overflowDelay == 0) {
            TIMA = TMA;
            setInterrupt();
            overflowPending = false;
        }
}
}

int timer::setClockBit(){
/*
    Base off of DIV 16384 hz timing and 16 bit output
    Because DIV increments at 16384 hz, we do that using an 8 bit counter (lower 2 nibbles of div_counter),
    and a 8 bit read (upper 2 nibbles). Therefore, any division or multiple of 16384 is adjusted using
    choosing the relevant bit relative to speed, and watching when it falls edge
*/  
    switch(TAC & 0x3) { // bits 0/1 select clock speed
        case (0):
            return 9; // 4096 hz 
        case (1):
            return 3; // 262144 hz
        case (2):
            return 5; // 65536 hz
        case (3):
            return 7; // 16384 hz
        default: return 9;
    }
}

void timer::setInterrupt(){
    uint8_t IF = MMU.readMem(0xFF0F);
    MMU.writeMem(IF | 0x4, 0xFF0F);
}

