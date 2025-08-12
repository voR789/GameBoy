#include "ppu.h"
#include "mmu.h"
#include "dma.h"

ppu::ppu(mmu& mmu_ref, timer& timer_ref, dma& dma_ref) : MMU(mmu_ref), TIMER(timer_ref), DMA(dma_ref){
    MMU.linkPPU(this);
    LCDC = 0x91;  // Power-on default 
    STAT = 0x85;  // Default status register value
    SCY  = 0x00;
    SCX  = 0x00;
    LY   = 0x00;  // Always starts at 0
    LYC  = 0x00;
    DMA_reg  = 0x00;
    BGP  = 0xFC;  // Default BG palette (often 0xFC or 0xE4)
    OBP0 = 0xFF;  // Default sprite palette 0
    OBP1 = 0xFF;  // Default sprite palette 1
    WY   = 0x00;  // Window Y position offscreen at start
    WX   = 0x00;  // Window X position offscreen at start
}

/* LCDC Bitmap: 
    7: LCD & PPU Enable
    6: Window tile map
    5: Window enable 
    4: BG and Window Tile data select
    3: BG tile map
    2: OBJ size
    1: OBJ Enable
    0: BG & Window Enable
*/
uint8_t ppu::readLCDC(){
    return LCDC;
}

void ppu::writeLCDC(uint8_t byte){
    LCDC = byte;
}

/* STAT Bitmap:
    7: LYC=LY Interrupt Enable
    6: Mode 2 OAM Interrupt Enable
    5: Mode 1 VBlank Interrupt Enable
    4: Mode 0 HBlank Interrupt Enable
    3: Coincidence Flag (LY == LYC)
    2: Mode Flag (bit 1)
    1: Mode Flag (bit 0)
    0: Unused (always 0)
*/
uint8_t ppu::readSTAT(){
    return STAT;
}

void ppu::writeSTAT(uint8_t byte){
    byte &= ~0x7; // Mask out read only bits
    STAT = byte | (STAT & 0x7); 
} 

uint8_t ppu::readLY(){
    return 0;
}

void ppu::writeDMA(uint8_t byte){
    DMA.startDMA(byte);
}