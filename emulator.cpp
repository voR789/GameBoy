#include <iostream>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"

int main(){
    mmu MMU;
    ppu PPU;
    timer TIMER(MMU);
    cpu CPU(MMU, PPU, TIMER);
    MMU.loadGame("01-special.gb");
    CPU.setPC(0x0100);
    while(true) {
        TIMER.tick(CPU.step());
    } 

    return 0;
}