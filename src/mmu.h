#ifndef MMU_H
#define MMU_H
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include "mbc.h"

// Forward Declaration to avoid circular import issues
class timer;
class ppu;
class serial;

class mmu{
    private:
        // ROM
        std::vector<uint8_t> ROM;
        std::vector<uint8_t> SRAM;
        uint8_t cartType;
        uint8_t romSize;
        uint8_t ramSize;
        // Static Memory Map
        uint8_t VRAM[8192];
        uint8_t WRAM[8192];
        uint8_t ECHO_RAM[7680]; // Mirrors WRAM
        uint8_t OAM[160];
        uint8_t IO_REGISTERS[128]; // TODO: Next to eliminate
        uint8_t HRAM[127];

        // Interrupt Flags
        uint8_t IF;
        uint8_t IE;

        // MBC
        std::unique_ptr<MBC> mbc;

        // Serial Pointer
        serial* SERIAL;

        // Timer Pointer
        timer* TIMER;

        // PPU Pointer
        ppu* PPU;
        bool dmaTransfer;
    public:
        mmu();
        void clearMem();
        uint8_t readMem(uint16_t index);
        void writeMem(uint8_t byte, uint16_t address);       
        void loadGame(const std::string& filename);
        
        // Serial Output mutual connection
        void linkSERIAL(serial* serial_);

        // TIMER mutual connection
        void linkTIMER(timer* timer_);

        // PPU mutual connection
        void linkPPU(ppu* ppu_);
        void startDMA(uint8_t byte);
};

#endif