#ifndef EMULATOR_H
#define EMULATOR_H
#include <iostream>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"
class emulator{
    private:
        cpu CPU;
        mmu MMU;
        ppu PPU;
    public:
        
};

#endif