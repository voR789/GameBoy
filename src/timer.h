#ifndef TIMER_H
#define TIMER_H
#include <cstdint>
#include "mmu.h"

class timer{
    private:
        mmu& MMU;
        uint16_t div_counter;
        uint8_t tima_counter;
        bool stopped;
        
    public:
        timer(mmu& mmu_ref);
        void tick(int cycles);
        void stopCall();
        void printTimerDebug();
};

#endif