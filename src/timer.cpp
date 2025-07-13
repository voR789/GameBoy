#include "timer.h"
#include "mmu.h"
#include <cstdint>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstring>
#include <bitset>

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

    MMU.setDiv_Counter(div_counter);

    // printTimerDebug();

    uint8_t TAC = MMU.readMem(REG_TAC);
    bool enabled = TAC & 0x4; // bit 2 reads enable
    if (enabled)
    {                         
        int clockSelect = TAC & 0x3; // isolate last 2 bits of TAC
        int TCycles;
        switch (clockSelect)
        {
        case (0):
            TCycles = 1024;
            break;
        case (1):
            TCycles = 16;
            break;
        case (2):
            TCycles = 64;
            break;
        case (3):
            TCycles = 256;
            break;
        } // 1 M-cycle = 4 T-cycles
        
        tima_counter += cycles;

        while(tima_counter >= TCycles){ // use while loop apporach if tima_counter overflows multiple times over
            uint8_t TIMA = MMU.readMem(REG_TIMA);
            tima_counter -= TCycles; 
            if(TIMA == 0xFF){
                MMU.writeMem(MMU.readMem(REG_TMA), REG_TIMA);
                uint8_t IF = MMU.readMem(0xFF0F);
                //std::cout << "TIMA Overflow, IF: " << std::bitset<8>(IF) << '\n';
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

void timer::printTimerDebug()
{
    uint8_t DIV = div_counter >> 8;
    uint8_t TIMA = MMU.readMem(REG_TIMA);
    uint8_t TMA = MMU.readMem(REG_TMA);
    uint8_t TAC = MMU.readMem(REG_TAC);
    uint8_t IF = MMU.readMem(0xFF0F);

    std::cout << "DIV:  0x" << std::hex << +DIV
              << " (raw counter: " << std::dec << div_counter << ")\n";
    std::cout << "TIMA: 0x" << std::hex << +TIMA << "\n";
    std::cout << "TMA:  0x" << std::hex << +TMA << "\n";
    std::cout << "TAC:  0x" << std::hex << +TAC
              << " (enabled: " << ((TAC & 0x04) ? "yes" : "no")
              << ", clock: " << (TAC & 0x03) << ")\n";


}
