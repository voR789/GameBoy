#include <iostream>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"

int main(){
    mmu MMU;
    ppu PPU;
    timer TIMER;
    cpu CPU(MMU, PPU, TIMER);
    MMU.loadGame("03-op sp,hl.gb");
    CPU.setPC(0x0100);
    while(true) {
        CPU.step();
}

    return 0;
}