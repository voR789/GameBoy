#ifndef MBC_H
#define MBC_H
#include <vector>
#include <cstdint>
#include <chrono>
// Abstract MBC class for multiple implementations
# pragma once
class MBC {
    public:
        virtual ~MBC() = default;

        virtual uint8_t readROM(uint16_t addr) = 0;
        virtual void writeROM(uint8_t byte, uint16_t addr) = 0;
        virtual uint8_t readRAM(uint16_t addr) = 0;
        virtual void writeRAM(uint8_t byte, uint16_t addr) = 0;
        virtual void reset() = 0;
};

class MBC1 : public MBC {
    private:
        uint8_t ram_enable;
        uint8_t rom_bank;
        uint8_t ram_bank_or_upper_rom_bank;
        uint8_t bank_mode;
        const std::vector<uint8_t>& ROM;
        std::vector<uint8_t>& SRAM;
    public:
        MBC1(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram);

        uint8_t readROM(uint16_t addr) override;
        void writeROM(uint8_t byte, uint16_t addr) override;
        uint8_t readRAM(uint16_t addr) override;
        void writeRAM(uint8_t byte, uint16_t addr) override;
        void reset() override;
};

class MBC2 : public MBC {
    private:
        uint8_t ram_enable;
        uint8_t rom_bank;
        const std::vector<uint8_t>& ROM;
        std::vector<uint8_t>& SRAM;
    public:
        MBC2(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram);

        uint8_t readROM(uint16_t addr) override;
        void writeROM(uint8_t byte, uint16_t addr) override;
        uint8_t readRAM(uint16_t addr) override;
        void writeRAM(uint8_t byte, uint16_t addr) override;
        void reset() override;
};

class MBC3 : public MBC {
    private:
        // RTC Registers
        uint8_t rtc_s;
        uint8_t rtc_m;
        uint8_t rtc_h;
        uint8_t rtc_dl;
        uint8_t rtc_dh;
        // Emulator Clock 
        uint64_t accumulatedTime;
        std::chrono::steady_clock::time_point startTime;

        // Memory Registers
        uint8_t ram_and_timer_enable;
        uint8_t rom_bank;
        uint8_t ram_bank_or_rtc_select;
        uint8_t latch_clock;

        const std::vector<uint8_t>& ROM;
        std::vector<uint8_t>& SRAM;
    public:
        
        MBC3(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram);

        uint8_t readROM(uint16_t addr) override;
        void writeROM(uint8_t byte, uint16_t addr) override;
        uint8_t readRAM(uint16_t addr) override;
        void writeRAM(uint8_t byte, uint16_t addr) override;
        void reset() override;
};

class MBC5 : public MBC {
    private:
        uint8_t ram_enable;
        uint8_t rom_bank_lower; // Lower 8 bits of ROM Bank
        uint8_t rom_bank_upper; // 9th bit of ROM Bank
        uint8_t ram_bank;
        // no rumble :(;
        const std::vector<uint8_t>& ROM;
        std::vector<uint8_t>& SRAM;
    public:
        MBC5(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram);

        uint8_t readROM(uint16_t addr) override;
        void writeROM(uint8_t byte, uint16_t addr) override;
        uint8_t readRAM(uint16_t addr) override;
        void writeRAM(uint8_t byte, uint16_t addr) override;
        void reset() override;
};

class noMBC : public MBC {
    private:
        const std::vector<uint8_t>& ROM;
        std::vector<uint8_t>& SRAM;
    public:
        noMBC(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram);
      
        uint8_t readROM(uint16_t addr) override;
        void writeROM(uint8_t byte, uint16_t addr) override;
        uint8_t readRAM(uint16_t addr) override;
        void writeRAM(uint8_t byte, uint16_t addr) override;
        void reset() override;  
};
#endif