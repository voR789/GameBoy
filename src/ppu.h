#ifndef PPU_H
#define PPU_H
#include "mmu.h"
#include "timer.h"
#include "dma.h"

#define REG_ACCESS(name)                        \
    uint8_t read##name() const { return name; }  \
    void write##name(uint8_t byte) { name = byte; }

#pragma once
class ppu{
    private:
        mmu& MMU;
        timer& TIMER;
        dma& DMA;

        uint8_t VRAM[8192];
        uint8_t OAM[160];

        enum mode {
            HBlank_mode = 0,
            VBlank_mode = 1,
            OAM_mode = 2,
            Transfer_mode = 3
        };

        mode render_mode;
        uint8_t LCDC;
        uint8_t STAT;
        uint8_t SCY;
        uint8_t SCX;
        uint8_t LY;
        uint8_t LYC;
        uint8_t DMA_reg;
        uint8_t BGP;
        uint8_t OBP0;
        uint8_t OBP1;
        uint8_t WY;
        uint8_t WX;

        // rendering logic
        int x;
        bool g;
        bool b;
    public:
        ppu(mmu& mmu_ref, timer& timer_ref, dma& dma_ref);

        // VRAM and OAM
        uint8_t readVRAM(uint16_t addr);
        void writeVRAM(uint8_t byte, uint16_t addr);
        uint8_t readOAM(uint16_t addr);
        void writeOAM(uint8_t byte, uint16_t addr);
        void dmaWriteOAM(uint8_t byte, uint16_t addr);

        // Registers with no special quirks
        // Using macro defined at top
        REG_ACCESS(SCY);
        REG_ACCESS(SCX);
        REG_ACCESS(LYC);
        REG_ACCESS(BGP);
        REG_ACCESS(OBP0);
        REG_ACCESS(OBP1);
        REG_ACCESS(WY);
        REG_ACCESS(WX);
        
        // More complex access
        uint8_t readLCDC();
        void writeLCDC(uint8_t byte);
        uint8_t readSTAT();
        void writeSTAT(uint8_t byte);
        uint8_t readLY();
        void writeDMA(uint8_t byte);

        void tick();
        // Rendering
        struct pixel{
            uint8_t color; // 0-3
            uint8_t palette; 
            uint8_t background_prio; 
        };
        struct pixel_Fetcher{
            pixel tile_row[8];
            int step = 0;
            uint16_t tile_index;
            uint16_t map_addr;
            uint8_t fifo_offset = 0;
        };
};
#endif