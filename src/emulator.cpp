#include <iostream>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"

int main(){
    mmu MMU;
    ppu PPU;
    timer TIMER(MMU);
    cpu CPU(MMU, PPU, TIMER);
    MMU.loadGame("2-interrupts.gb");
    int cycle;
    int counter = 10000;
    while(true) {
        cycle = CPU.step();
        TIMER.tick(cycle);

        if(counter < 0){
            CPU.triggerVBLankInterrupt();
            counter = 10000;
        } else{
            counter -= cycle;
        }
    } 

    return 0;
}