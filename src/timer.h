#ifndef TIMER_H
#define TIMER_H
#include <cstdint>
#include "mmu.h"

class timer{
    private:
        mmu& MMU;
        bool stopped;
        int totalCycles;
    public:
        timer(mmu& mmu_ref);
        void tick(int cycles);
        void stopCall();
        void printTimerDebug();
        void incTIMA();
        int readTAC();
};

#endif