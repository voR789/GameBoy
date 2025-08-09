#ifndef DMA_H
#define DMA_H
#include <cstdint>
#include "mmu.h"
#include "ppu.h"

class dma{
    private:
        mmu& MMU;
        ppu& PPU;

        uint16_t start_addr;
        int index;
    public:
        dma(mmu& mmu_ref, ppu& ppu_ref);
        void startDMA(uint8_t byte);
        void tick();
};

#endif