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

timer::timer(mmu &MMU_REF) : MMU(MMU_REF), stopped(false) {}

void timer::tick(int cycles) {
    if (stopped) {
        return;
    }

    // Part of DIV logic lives in memory
    uint16_t oldDiv = MMU.getDivCounter();
    MMU.addDivCounter(cycles);
    uint16_t newDiv = MMU.getDivCounter();

    uint8_t TAC     = MMU.readMem(REG_TAC);
    bool    enabled = TAC & 0x4;  // bit 2 reads enable
    if (enabled) {
        //printTimerDebug();
        totalCycles += cycles;
        int clockSelect = TAC & 0x3;  // isolate last 2 bits of TAC
        int bit;
        switch (clockSelect) {
            /*
                Base off of DIV 16384 hz timing and 16 bit output
                Because DIV increments at 16384 hz, we do that using an 8 bit counter (lower 2 nibbles of div_counter),
               and a 8 bit storer (upper 2 nibbles). Therefore, any division or multiple of 16384 is adjusted using
               choosing the relevant bit relative to speed, and watching when it falls edge
            */
            case (0):
                bit = 9;
                break;  // 4096 hz
            case (1):
                bit = 3;
                break;  // 262144 hz
            case (2):
                bit = 5;
                break;  // 65536 hz
            case (3):
                bit = 7;
                break;  // 16384 hz
        }
        for (int i = 0; i < cycles; ++i) {
            uint16_t oldDiv = MMU.getDivCounter();
            MMU.addDivCounter(1);
            uint16_t newDiv = MMU.getDivCounter();
            uint8_t oldBit = (oldDiv >> bit) & 1;
            uint8_t newBit = (newDiv >> bit) & 1;

            if (oldBit == 1 && newBit == 0) {
                uint8_t TIMA = MMU.readMem(REG_TIMA);
                if (TIMA == 0xFF) {
                    //std::cin.get();
                    uint8_t TMA = MMU.readMem(REG_TMA);
                    MMU.writeMem(TMA, REG_TIMA);
                    uint8_t IF = MMU.readMem(0xFF0F);
                    MMU.writeMem(IF | 0x4, 0xFF0F);
                } else {
                    MMU.writeMem(TIMA + 1, REG_TIMA);
                }
            }
        }
    }
}

void timer::stopCall() { stopped = true; }

void timer::printTimerDebug() {
    uint8_t DIV  = MMU.getDivCounter();
    uint8_t TIMA = MMU.readMem(REG_TIMA);
    uint8_t TMA  = MMU.readMem(REG_TMA);
    uint8_t TAC  = MMU.readMem(REG_TAC);
    uint8_t IF   = MMU.readMem(0xFF0F);

    std::cout << "Elapsed M - Cycles: " << std::dec << (int)totalCycles/4 << '\n';
    std::cout << "IF: 0x" << std::hex << (int)IF << "\n";
    std::cout << "DIV:  0x" << std::hex << +DIV << "\n";
    std::cout << "TIMA: 0x" << std::hex << +TIMA << "\n";
    std::cout << "TMA:  0x" << std::hex << +TMA << "\n";
    std::cout << "TAC:  0x" << std::hex << +TAC << " (enabled: " << ((TAC & 0x04) ? "yes" : "no")
              << ", clock: " << (TAC & 0x03) << ")\n";
}
