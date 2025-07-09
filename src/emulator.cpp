#include <iostream>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"

int main(){
    mmu MMU;
    ppu PPU;
    timer TIMER(MMU);
    cpu CPU(MMU, PPU, TIMER);
    // MMU.loadGame("tests/01-special.gb"); // - passed
    MMU.loadGame("tests/02-interrupts.gb");
    // MMU.loadGame("tests/03-op sp,hl.gb"); // - passed
    // MMU.loadGame("tests/04-op r,imm.gb"); // - passed
    // MMU.loadGame("tests/05-op rp.gb"); // - passed
    // MMU.loadGame("tests/06-ld r,r.gb"); // - passed
    // MMU.loadGame("tests/07-jr,jp,call,ret,rst.gb"); // - passed
    // MMU.loadGame("tests/08-misc instrs.gb"); // - passed
    // MMU.loadGame("tests/09-op r,r.gb"); // - passed
    // MMU.loadGame("tests/10-bit ops.gb"); // - passed
    // MMU.loadGame("tests/11-op a,(hl).gb"); // - passed
    int cycle;
    int counter = 10000;

    int stepper;
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