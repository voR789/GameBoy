#ifndef DMA_H
#define DMA_H
#include <cstdint>
#include "mmu.h"

class dma{
    private:
        mmu& MMU;

        uint16_t start_addr;
        int index;
    public:
        dma(mmu& mmu_ref);
        void startDMA(uint8_t byte);
        void tick();
};

#endif