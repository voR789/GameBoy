#ifndef MMU_H
#define MMU_H
#include <cstdint>
#include <string>
#include <vector>
class mmu{
    private:
        // ROM
        std::vector<uint8_t> ROM;
        uint8_t cartType;
        uint8_t romSize;
        uint8_t ramSize;
        uint16_t mask;
        bool largeBankMode;
        uint16_t BANK;
        uint8_t MBC_REG[4];
        std::vector<uint8_t> SRAM;
        // RAM Enable, ROM BANK Number, RAM Bank Number or ROM BANK Number (Upper), Banking Mode

        // Memory Map
        uint8_t ROM_0[16384];
        uint8_t SWITCH_ROM[16384]; // Bankable
        uint8_t VRAM[8192];
        uint8_t ERAM[8192]; // Bankable
        uint8_t WRAM[8192];
        uint8_t ECHO_RAM[7680]; // Mirrors WRAM
        uint8_t OAM[160];
        // uint8_t NAN[96] - unused memory
        uint8_t IO_REGISTERS[128];
        uint8_t HRAM[127];
        uint8_t IE;

        // Timer
        uint16_t divCounter;
    public:
        mmu();
        void clearMem();
        void addDivCounter(int cycles);
        uint16_t getDivCounter();
        uint8_t readMem(uint16_t index);
        void writeMem(uint8_t byte, uint16_t address);       
        void loadGame(const std::string& filename);
        uint8_t handleROMRead(uint16_t index);
        uint8_t handleRAMRead(uint16_t index);
        void handleRAMWrite(uint8_t byte, uint16_t index);
        void write_MBC_REG(uint8_t byte, uint16_t index);
        void recalculateBank();
};

#endif