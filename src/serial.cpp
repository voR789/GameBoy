#include "serial.h"
#include "mmu.h"
#include <iostream>
#include <cstdint>

serial::serial(mmu& mmu_ref): MMU(mmu_ref), SB(0x0), SC(0x7E) {
    MMU.linkSERIAL(this);
}

uint8_t serial::readSB(){
    return SB;
}

uint8_t serial::readSC(){
    return SC;
}

void serial::writeSB(uint8_t byte){
    SB = byte;
}

void serial::writeSC(uint8_t byte){
    if(byte & 0x80){
        std::cout << "" << static_cast<char>(SB);
        std::cout.flush();
        // "Clear bit 7" and trigger Serial Interrupt
        SC = (byte & ~0x80);
        setInterrupt();
    }
}

void serial::setInterrupt(){
    uint8_t IF = MMU.readMem(0xFF0F);
    MMU.writeMem(IF | 0x8, 0xFF0F);
}
