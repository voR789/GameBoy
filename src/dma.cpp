#include "dma.h"
#include "mmu.h"
#include <cstdint>
#include <iostream>

dma::dma(mmu& mmu_ref)
    : MMU(mmu_ref), start_addr(0x0000), index(-1) {}

void dma::startDMA(uint8_t byte){
    start_addr = byte << 8;
    index = 0;
    MMU.setDMAFlag();
}

void dma::tick(){
    // If index is non-negative, that means DMA has started
    if(index >= 0){
        uint16_t addr = start_addr + index;
        uint8_t data = MMU.readMem(addr);
        MMU.writeOAM(data, 0xFE00 + index);
        index++;
    }

    // End DMA
    if(index == 160){
        MMU.clearDMAFlag();
        index = -1;
    }
}