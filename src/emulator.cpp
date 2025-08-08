#include <iostream>

#include "cpu.h"
#include "mmu.h"
#include "ppu.h"
#include "timer.h"
#include "serial.h"

int main() {
    // TODO: Verify power on starting vals when done, add stopped logic
    mmu   MMU;
    serial SERIAL(MMU);
    timer TIMER(MMU);
    ppu   PPU(MMU, TIMER);
    cpu   CPU(MMU, PPU, TIMER);
    // MMU.loadGame("tests/01-special.gb"); // - passed
    // MMU.loadGame("tests/02-interrupts.gb"); // - passed
    // MMU.loadGame("tests/03-op sp,hl.gb"); // - passed
    // MMU.loadGame("tests/04-op r,imm.gb"); // - passed
    // MMU.loadGame("tests/05-op rp.gb"); // - passed
    // MMU.loadGame("tests/06-ld r,r.gb"); // - passed
    // MMU.loadGame("tests/07-jr,jp,call,ret,rst.gb"); // - passed
    // MMU.loadGame("tests/08-misc instrs.gb"); // - passed
    // MMU.loadGame("tests/09-op r,r.gb"); // - passed
    // MMU.loadGame("tests/10-bit ops.gb"); // - passed
    // MMU.loadGame("tests/11-op a,(hl).gb"); // - passed

    MMU.loadGame("tests/cpu_instrs.gb");  // - passed!
    // MMU.loadGame("tests/instr_timing.gb"); // passed!
    // MMU.loadGame("tests/halt_bug.gb"); // - do ppu first
    // MMU.loadGame("tests/interrupt_time.gb"); - do ppu first
    int  MCycles;
    bool emulating = true;
    while (emulating) {
        MCycles = CPU.step();
        for (int i = 0; i < (MCycles * 4); i++) {
            TIMER.tick();
            // TODO: PPU.tick();
        }
    }

    return 0;
}