#ifndef PPU_H
#define PPU_H
#include "mmu.h"
#include "timer.h"
#define REG_ACCESS(name)                        \
    uint8_t read##name() const { return name; }  \
    void write##name(uint8_t byte) { name = byte; }

class ppu{
    private:
        mmu& MMU;
        timer& TIMER;
        enum mode {
            HBlank = 0,
            VBlank = 1,
            OAM = 2,
            Transfer = 3
        };
        uint8_t LCDC;
        uint8_t STAT;
        uint8_t SCY;
        uint8_t SCX;
        uint8_t LY;
        uint8_t LYC;
        uint8_t DMA;
        uint8_t BGP;
        uint8_t OBP0;
        uint8_t OBP1;
        uint8_t WY;
        uint8_t WX;
    public:
        ppu(mmu& mmu_ref, timer& timer_ref);
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
};
#endif