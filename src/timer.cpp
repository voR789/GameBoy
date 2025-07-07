#include "timer.h"
#include "mmu.h"
#include <cstdint>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstring>

// Timer registers are at:
#define REG_DIV 0xFF04
#define REG_TIMA 0xFF05
#define REG_TMA 0xFF06
#define REG_TAC 0xFF07

timer::timer(mmu &MMU_REF) : MMU(MMU_REF), div_counter(0), tima_counter(0), stopped(false) {}

void timer::tick(int cycles)
{
    if (stopped)
    {
        return;
    }

    if (MMU.getDivReset())
    {
        div_counter = 0;
        MMU.clearDivReset();
    }
    else
    {
        div_counter += cycles;
    }
    MMU.writeMem((div_counter >> 8), REG_DIV); // gives the amount of times 256 div_counter

    uint8_t TAC = MMU.readMem(REG_TAC);
    bool enabled = TAC & 0x4;
    if (enabled)
    {                         // bit 2 reads enable
        std::cout << "yooooo" << std::endl;
        int clockSelect = TAC & 0x3; // isolate last 2 bits of TAC
        int MCycles;
        switch (clockSelect)
        {
        case (0):
            MCycles = 256;
            break;
        case (1):
            MCycles = 4;
            break;
        case (2):
            MCycles = 16;
            break;
        case (3):
            MCycles = 64;
            break;
        } // 1 M-cycle = 4 T-cycles
        int TCycles = MCycles * 4;
        uint8_t TMA = MMU.readMem(REG_TMA);
        
        tima_counter += cycles;

        while(tima_counter >= TCycles){ // use while loop apporach if tima_counter overflows multiple times over
            uint8_t TIMA = MMU.readMem(REG_TIMA);
            tima_counter -= TCycles; 
            if(TIMA == 0xFF){
                MMU.writeMem(TMA, REG_TIMA);
                uint8_t IF = MMU.readMem(0xFF0F);
                MMU.writeMem(IF | 0x4, 0xFF0F); // set timer interrupt flag
            } else{
                MMU.writeMem(TIMA + 1, REG_TIMA);
            }
        }
    }
    // push to mem
}

void timer::stopCall()
{
    stopped = true;
}