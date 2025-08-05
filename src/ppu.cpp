#include "ppu.h"

#include "mmu.h"

ppu::ppu(mmu& MMU_ref) : MMU(MMU_ref) {
    MMU.linkPPU(this);
    LCDC = 0x91;  // Power-on default 
    STAT = 0x85;  // Default status register value
    SCY  = 0x00;
    SCX  = 0x00;
    LY   = 0x00;  // Always starts at 0
    LYC  = 0x00;
    DMA  = 0x00;
    BGP  = 0xFC;  // Default BG palette (often 0xFC or 0xE4)
    OBP0 = 0xFF;  // Default sprite palette 0
    OBP1 = 0xFF;  // Default sprite palette 1
    WY   = 0x00;  // Window Y position offscreen at start
    WX   = 0x00;  // Window X position offscreen at start
}

// LCDC:
/* LCDC Bitmap: 
    7: LCD & PPU Enable, 6: Window tile map, 5: Window enable 4: BG and Window Tile data select, 3: BG tile map, 2: OBJ size, 1: OBJ Enable, 0: BG & Window Enable
*/
uint8_t ppu::readLCDC(){

}

void ppu::writeLCDC(uint8_t byte){

}

uint8_t ppu::readSTAT(){

}

void ppu::writeSTAT(uint8_t byte){

}

uint8_t readLY(){

}

void writeDMA(){

}