#include "mmu.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <cstddef>

#include "mbc.h"
#include "ppu.h"
#include "serial.h"
// Timer registers are at:
#define REG_DIV 0xFF04
#define REG_TIMA 0xFF05
#define REG_TMA 0xFF06
#define REG_TAC 0xFF07

mmu::mmu() : PPU(nullptr), TIMER(nullptr), SERIAL(nullptr) {
    reset(); 
}

void mmu::reset() {
    loadBootROM();
    memset(VRAM, 0, sizeof(VRAM));
    memset(WRAM, 0, sizeof(WRAM));
    memset(OAM, 0, sizeof(OAM));
    memset(HRAM, 0, sizeof(HRAM));
    IE = 0x0;
    IF = 0xE1;
    bootROMEnable = true;
    dmaFlag = false;
}

uint8_t mmu::readMem(uint16_t index) {
    if(dmaFlag){
        if(index >= 0xFF80 && index <= 0xFFFE){ // ONLY Allow HRAM access in DMA
            return HRAM[index - 0xFF80];
        } else{
            return 0xFF; // Don't allow any other accesses
        }
    }
    // ROM
    else if (index <= 0x7FFF) {
        if(bootROMEnable){
            if(index <= 0x100){
                return readBootROM(index);
            } 
            else if(index <= 0x133){
                // Boot ROM uses header for logo
                return mbc->readROM(index);
            }
        }
        return mbc->readROM(index);
    }
    // Video RAM
    else if (index <= 0x9FFF) {
        return VRAM[index - 0x8000];  
    }
    // External (MBC) RAM
    else if (index <= 0xBFFF) {
        return mbc->readRAM(index);
    }
    // Work RAM
    else if (index <= 0xDFFF) {
        return WRAM[index - 0xC000];  
    }
    // Echo RAM - Mirrors WRAM
    else if (index <= 0xFDFF) {
        return WRAM[index - 0xE000]; 
    }
    // Object Attribute Memory
    else if (index <= 0xFE9F) {
        return OAM[index - 0xFE00]; 
    }
    // Unused Memory Map
    else if (index <= 0xFEFF) {
        return 0;
    }
    // I/O Registers
    else if (index <= 0xFF7F) {
        switch (index) {  
            case 0xFF00:
                return 0xCF;  // TODO: JOYP (no buttons pressed)
            case 0xFF01:
                return SERIAL->readSB();  // SB (Serial Transfer Data)
            case 0xFF02:
                return SERIAL->readSC();  // SC (Serial Transfer Control)

            // Timer Registers
            case 0xFF04:
                return TIMER->readDIV();  // DIV (Counts at 16384 Hz)
            case 0xFF05:
                return TIMER->readTIMA();  // TIMA (Timer)
            case 0xFF06:
                return TIMER->readTMA();  // TMA (Timer reset value)
            case 0xFF07:
                return TIMER->readTAC();  // TAC (Controls Timer)

            case 0xFF0F:
                return IF;  // IF (Interrupt Flag)

            // cases 0xFF10 - FF3F -> audio (not handled)

            // PPU Registers
            case 0xFF40:
                return PPU->readLCDC();  // LCD Control
            case 0xFF41:
                return PPU->readSTAT();  // STAT
            case 0xFF42:
                return PPU->readSCY();  // Background Y pos
            case 0xFF43:
                return PPU->readSCX();  // Background X pos
            case 0xFF44:
                return PPU->readLY();  // LY
            case 0xFF45:
                return PPU->readLYC();  // LYC
            case 0xFF46:
                return 0xFF;  // DMA WO
            case 0xFF47:
                return PPU->readBGP();  // BGP
            case 0xFF48:
                return PPU->readOBP0();  // Object Palette 0
            case 0xFF49:
                return PPU->readOBP1();  // Object Palette 1
            case 0xFF4A:
                return PPU->readWY();  // WY
            case 0xFF4B:
                return PPU->readWX();  // WX
            case 0xFF50:
                return 0xFF;  // return trash as boot rom control is write only
        }
    }                     
    // High RAM
    else if (index <= 0xFFFE) {
        return HRAM[index - 0xFF80];
    }
    // Interrupt Enable
    else if (index == 0xFFFF) {
        return IE;
    }
    else {
        return 0;
    }
    return 0;
}

void mmu::writeMem(uint8_t byte, uint16_t index) {
    if(dmaFlag){
        if(index >= 0xFF80 && index <= 0xFFFE){ // ONLY Allow HRAM access in DMA
            HRAM[index - 0xFF80] = byte;
        } else{
            return; // Don't allow any other accesses
        }
    }
    // ROM
    else if (index <= 0x7FFF) {
        // writes to this area are not allowed traditionally, instead used as writes to MBC registers
        if(bootROMEnable){
            return;
        }
        mbc->writeROM(byte, index);
        return;
    }
    // Video RAM
    else if (index <= 0x9FFF) {
        VRAM[index - 0x8000] = byte;
        return;
    }
    // External (MBC) RAM
    else if (index <= 0xBFFF) {
        mbc->writeRAM(byte, index);
        return;
    }
    // Work RAM
    else if (index <= 0xDFFF) {
        WRAM[index - 0xC000] = byte;
        return;
    }
    // Echo RAM
    else if (index <= 0xFDFF) {
        WRAM[index - 0xE000] = byte; 
        return;
    }
    // Object Attribute Memory  
    else if (index <= 0xFE9F) {
        OAM[index - 0xFE00] = byte;
        return;
    }
    // Unused Memory
    else if (index <= 0xFEFF) {
        return;
    }
    // I/O Registers
    else if (index <= 0xFF7F) {
        switch (index) {  
            // TODO: Joypad input

            // Serial Transfer
            case 0xFF01:  // SB (Serial Output Data)
                SERIAL->writeSB(byte);
                break;
            case 0xFF02:  // SC (Serial Output Enable) - Actual wire functionality stubbed
                SERIAL->writeSC(byte);
                break;

            // Timer Registers
            case 0xFF04:
                TIMER->writeDIV(byte);  // Represents DIV or 0xFF04
                break;
            case 0xFF05:
                TIMER->writeTIMA(byte);  // TIMA
                break;
            case 0xFF06:
                TIMER->writeTMA(byte);  // TMA (Timer Overflow Value)
                break;
            case 0xFF07:
                TIMER->writeTAC(byte);  // TAC (Selects timer speed off bits)
                break;

            case 0xFF0F:
                IF = byte;  // Interrupt Flag
                break;

            // Audio stubbed (0xFF10-0xFF3F)

            // PPU Registers
            case 0xFF40:
                PPU->writeLCDC(byte);  // LCD Control
                break;
            case 0xFF41:
                PPU->writeSTAT(byte);  // STAT
                break;
            case 0xFF42:
                PPU->writeSCY(byte);  // Background Y pos
                break;
            case 0xFF43:
                PPU->writeSCX(byte);  // Background X pos
                break;
            case 0xFF44:
                break;  // LY RO
            case 0xFF45:
                PPU->writeLYC(byte);  // LYC
                break;
            case 0xFF46:
                PPU->writeDMA(byte);  // DMA WO
                break;
            case 0xFF47:
                PPU->writeBGP(byte);  // BGP
                break;
            case 0xFF48:
                PPU->writeOBP0(byte);  // Object Palette 0
                break;
            case 0xFF49:
                PPU->writeOBP1(byte);  // Object Palette 1
                break;
            case 0xFF4A:
                PPU->writeWY(byte);  // WY
                break;
            case 0xFF4B:
                PPU->writeWX(byte);  // WX
                break;
            case 0xFF50:{
                std::cerr << "Escaped boot ROM";
                std::cin.get();
                bootROMEnable = false;
                break;
            }
        }
        return;
    } 
    // High RAM
    else if (index <= 0xFFFE) {
        HRAM[index - 0xFF80] = byte;
        return;
    }
    // Interrupt Enable 
    else if (index == 0xFFFF) {
        IE = byte & 0x1F;  // mask out unused bytes
        return;
    }
}

void mmu::loadGame(const std::string& filename) {  // Initialize MBC
    std::ifstream rom(filename, std::ios::binary);
    if (!rom) {
        throw std::runtime_error("Failed to open ROM: " + filename);
    }

    // Initialize ROM/MBC
    ROM      = std::vector<uint8_t>((std::istreambuf_iterator<char>(rom)), {});  // ROM vector
    cartType = ROM.at(0x147);
    ramSize  = ROM.at(0x149);

    // Initialize External RAM
    switch (ramSize) {
        case (0x0): {
            SRAM = std::vector<uint8_t>(8192);
            break;
        }
        case (0x1): {
            SRAM = std::vector<uint8_t>(8192);
            break;
        }
        case (0x2): {
            SRAM = std::vector<uint8_t>(8192);
            break;
        }
        case (0x3): {
            SRAM = std::vector<uint8_t>(4 * 8192);
            break;
        }
        case (0x4): {
            SRAM = std::vector<uint8_t>(16 * 8192);
            break;
        }
        case (0x5): {
            SRAM = std::vector<uint8_t>(8 * 8192);
            break;
        }
    }

    // Initialize MBC
    if (cartType == 0x0 || cartType == 0x8 || cartType == 0x9) {
        mbc = std::make_unique<noMBC>(ROM, SRAM);
    } else if (cartType >= 0x1 && cartType <= 0x3) {
        mbc = std::make_unique<MBC1>(ROM, SRAM);
    } else if (cartType == 0x5 || cartType == 0x6) {
        mbc = std::make_unique<MBC2>(ROM, SRAM);
    } else if (cartType >= 0xF && cartType <= 0x13) {
        mbc = std::make_unique<MBC3>(ROM, SRAM);
    } else if (cartType >= 0x19 && cartType <= 0x1E) {
        mbc = std::make_unique<MBC5>(ROM, SRAM);
    }
}

// MBC1
MBC1::MBC1(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram)
    : ROM(rom), SRAM(sram), ram_enable(0), rom_bank(1), ram_bank_or_upper_rom_bank(1), bank_mode(0) {}

uint8_t MBC1::readROM(uint16_t addr) {
    int rom_address;
    if (bank_mode == 0) {
        if (addr <= 0x3FFF) {  // Bank 0
            rom_address = addr;
        } else {
            rom_address = (rom_bank * 0x4000) + (addr - 0x4000);
        }
    } else {                   // Bank mode 1
        if (addr <= 0x3FFF) {  // Bank 0
            rom_address = ((ram_bank_or_upper_rom_bank << 5) * 0x4000) + addr;
        } else {
            rom_address = (rom_bank * 0x4000) + (addr - 0x4000);
        }
    }
    return ROM.at(rom_address);
}

void MBC1::writeROM(uint8_t byte, uint16_t addr) {
    // Writes to ROM affect reg
    if (addr <= 0x1FFF) {
        // RAM Enable
        ram_enable = ((byte & 0xF) == 0xA);
    } else if (addr <= 0x3FFF) {
        // ROM Bank
        rom_bank = ((byte & 0x1F) == 0) ? 0x01 : (byte & 0x1F);  // If 0, convert to bank 1

        uint8_t romSize = ROM.at(0x0148);
        uint8_t mask    = (1 << (romSize + 1)) - 1;  // Formula maps romSize select to mask
        if (mask >= 0x1F && bank_mode == 0) {
            // Register maxes out at 5 bits, so for larger values we steal the RAM bank to make a 7 bit bank
            rom_bank = (ram_bank_or_upper_rom_bank << 5) | (rom_bank & 0x1F);
        } else {
            rom_bank = rom_bank & mask;
        }
    } else if (addr <= 0x5FFF) {
        // RAM Bank Number, or Upper ROM Bank Number
        ram_bank_or_upper_rom_bank = byte & 0x3;
    } else if (addr <= 0x7FFF) {
        // Banking Mode Select
        bank_mode = byte & 0x1;
    }
}

uint8_t MBC1::readRAM(uint16_t addr) {
    int sram_address;
    if (ram_enable) {
        if (bank_mode == 0) {
            sram_address = addr - 0xA000;
        } else {  // RAM Banking mode
            sram_address = (ram_bank_or_upper_rom_bank * 0x2000) + (addr - 0xA000);
        }
    } else {
        return 0xFF;
    }
    return SRAM.at(sram_address);
}

void MBC1::writeRAM(uint8_t byte, uint16_t addr) {
    int sram_address = 0xFF;  // Base trash val
    if (ram_enable) {
        if (bank_mode == 0) {
            sram_address = addr - 0xA000;
        } else {  // RAM Banking mode
            sram_address = (ram_bank_or_upper_rom_bank * 0x2000) + (addr - 0xA000);
        }
    } else {
        return;
    }
    SRAM.at(sram_address) = byte;
}

void MBC1::reset() {
    ram_enable                 = 0;
    rom_bank                   = 1;
    ram_bank_or_upper_rom_bank = 0;
    bank_mode                  = 0;
}

// MBC2
MBC2::MBC2(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram)
    : ROM(rom), SRAM(sram), ram_enable(0), rom_bank(1) {}

uint8_t MBC2::readROM(uint16_t addr) {
    int rom_address;
    if (addr <= 0x3FFF) {
        rom_address = addr;
    } else {  // Bank n
        rom_address = (rom_bank * 0x4000) + (addr - 0x4000);
    }
    return ROM.at(rom_address);
}

void MBC2::writeROM(uint8_t byte, uint16_t addr) {
    // Writes to ROM affect reg
    if (addr <= 0x3FFF) {
        // RAM Enable, ROM Bank Number
        if ((addr & 0x0100) == 0) {  // RAM Control (Bit 8 of addr clear)
            ram_enable = ((byte & 0xF) == 0xA);
        } else {  // ROM Control (Bit 8 of addr set)
            rom_bank = ((byte & 0xF) == 0) ? 0x1 : (byte & 0xF);
        }
    }
}

uint8_t MBC2::readRAM(uint16_t addr) {
    int sram_address;
    if (ram_enable) {
        if ((addr >= 0xA000) && (addr <= 0xA1FF)) {
            sram_address = addr - 0xA000;
        } else {  // behavior
            sram_address = (addr - 0xA000) % 512;
        }
    } else {
        return 0xFF;
    }
    return (SRAM.at(sram_address)) & 0xF;  // RAM is 4 bit
}

void MBC2::writeRAM(uint8_t byte, uint16_t addr) {
    int sram_address = 0xFF;  // Base trash val
    if (ram_enable) {
        if ((addr >= 0xA000) && (addr <= 0xA1FF)) {
            sram_address = addr - 0xA000;
        } else {  // Echo behavior
            sram_address = (addr - 0xA000) % 512;
        }
    } else {
        return;
    }
    SRAM.at(sram_address) = byte & 0xF;
}

void MBC2::reset() {
    ram_enable = 0;
    rom_bank   = 1;
}

// MBC3
MBC3::MBC3(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram)
    : ROM(rom),
      SRAM(sram),
      rtc_s(0),
      rtc_m(0),
      rtc_h(0),
      rtc_dl(0),
      rtc_dh(0),
      ram_and_timer_enable(0),
      rom_bank(1),
      ram_bank_or_rtc_select(0),
      latch_clock(0xFF) {}

uint8_t MBC3::readROM(uint16_t addr) {
    uint16_t rom_address;
    if (addr <= 0x3FFF) {
        rom_address = addr;
    } else {
        rom_address = (rom_bank * 0x4000) + (addr - 0x4000);
    }
    return ROM.at(rom_address);
}

void MBC3::writeROM(uint8_t byte, uint16_t addr) {
    // Writes to ROM affect reg
    if (addr <= 0x1FFF) {
        ram_and_timer_enable = ((byte & 0xF) == 0xA);
    } else if (addr <= 0x3FFF) {
        rom_bank = ((byte & 0x1F) == 0) ? 0x01 : (byte & 0x7F);  // If 0, convert to bank num 1
    } else if (addr <= 0x5FFF) {
        ram_bank_or_rtc_select = byte;  // 0x0 - 0x7 maps RAM, 0x8 - 0xC maps RTC
    } else if (addr <= 0x7FFF) {
        if (latch_clock == 0 && byte == 0x1) {
            // Update time
            auto now = std::chrono::steady_clock::now();
            if (!(rtc_dh & 0x40)) {  // Clock does not update upon halt
                accumulatedTime += std::chrono::duration_cast<std::chrono::seconds>(now - startTime)
                                       .count();  // "Save" amount of time passed
            }
            startTime = now;

            uint64_t timeMath = accumulatedTime;  // temp var to do time math
            uint16_t days     = timeMath / 86400;
            timeMath %= 86400;
            uint8_t hours = timeMath / 3600;
            timeMath %= 3600;
            uint8_t minutes = timeMath / 60;
            timeMath %= 60;
            uint8_t seconds = timeMath;

            // day counter carry flag
            if (days > 0x1FF) {
                rtc_dh |= 0x80;
            }
            rtc_dl = (days % 512) & 0xFF;
            rtc_dh = (((days % 512) & 0x100) >> 8) | (rtc_dh & 0xC0);  // New day bit plus old flags
            rtc_h  = hours;
            rtc_m  = minutes;
            rtc_s  = seconds;
        }
        latch_clock = byte;
    }
}

uint8_t MBC3::readRAM(uint16_t addr) {
    int sram_address;  // Base trash val
    if (ram_and_timer_enable) {
        if (ram_bank_or_rtc_select <= 0x07) {
            sram_address = (ram_bank_or_rtc_select * 0x2000) + (addr - 0xA000);
        } else if (ram_bank_or_rtc_select <= 0xC) {  // RTC Register mode
            switch (ram_bank_or_rtc_select) {
                case (0x08):
                    return rtc_s;
                case (0x09):
                    return rtc_m;
                case (0x0A):
                    return rtc_h;
                case (0x0B):
                    return rtc_dl;
                case (0x0C):
                    return rtc_dh;
            }
        } else {
            return 0xFF;  // Trash val
        }
    } else {
        return 0xFF;
    }
    return (SRAM.at(sram_address));
}

void MBC3::writeRAM(uint8_t byte, uint16_t addr) {
    int sram_address = 0xFF;  // Base trash val
    if (ram_and_timer_enable) {
        if (ram_bank_or_rtc_select <= 0x7) {
            sram_address          = (ram_bank_or_rtc_select * 0x2000) + (addr - 0xA000);
            SRAM.at(sram_address) = byte;
        } else if (ram_bank_or_rtc_select <= 0xC) {  // RTC Register mode
            // Update time
            auto now = std::chrono::steady_clock::now();
            if (!(rtc_dh & 0x40)) {  // Clock does not update upon halt
                accumulatedTime += std::chrono::duration_cast<std::chrono::seconds>(now - startTime)
                                       .count();  // "Save" amount of time passed
            }
            startTime = now;
            switch (ram_bank_or_rtc_select) {
                // rtc write logic
                case (0x08): {
                    rtc_s = byte;
                    break;
                }
                case (0x09): {
                    rtc_m = byte;
                    break;
                }
                case (0x0A): {
                    rtc_h = byte;
                    break;
                }
                case (0x0B): {
                    rtc_dl = byte;
                    break;
                }
                case (0x0C): {
                    rtc_dh = (byte & 0xC1);
                    break;  // only uses bits 0, 6, 7
                }
            }
        }
    } else {
        return;
    }
}

void MBC3::reset() {
    rtc_s                  = 0;
    rtc_m                  = 0;
    rtc_h                  = 0;
    rtc_dl                 = 0;
    rtc_dh                 = 0;
    ram_and_timer_enable   = 0;
    rom_bank               = 1;
    ram_bank_or_rtc_select = 0;
    latch_clock            = 0xFF;
}

// MBC5
MBC5::MBC5(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram)
    : ROM(rom), SRAM(sram), ram_enable(0), rom_bank_lower(1), rom_bank_upper(0), ram_bank(0) {}

uint8_t MBC5::readROM(uint16_t addr) {
    uint16_t rom_address;
    if (addr <= 0x3FFF) {
        rom_address = addr;
    } else {
        uint8_t rom_bank = (rom_bank_upper << 8) | rom_bank_lower;
        rom_address      = (rom_bank * 0x4000) + (addr - 0x4000);
    }
    return ROM.at(rom_address);
}

void MBC5::writeROM(uint8_t byte, uint16_t addr) {
    // Writes to ROM affect reg
    if (addr <= 0x1FFF) {
        ram_enable = ((byte & 0xF) == 0xA);
    } else if (addr <= 0x2FFF) {
        rom_bank_lower = byte;
    } else if (addr <= 0x3FFF) {
        rom_bank_upper = byte & 0x1;
    } else if (addr <= 0x5FFF) {
        ram_bank = byte & 0x0F;
    }
}

uint8_t MBC5::readRAM(uint16_t addr) {
    int sram_address;
    if (ram_enable) {
        sram_address = (ram_bank * 0x2000) + (addr - 0xA000);
    } else {
        return 0xFF;
    }
    return (SRAM.at(sram_address));
}

void MBC5::writeRAM(uint8_t byte, uint16_t addr) {
    int sram_address;
    if (ram_enable) {
        sram_address = (ram_bank * 0x2000) + (addr - 0xA000);
    } else {
        return;
    }
    SRAM.at(sram_address) = byte;
}

void MBC5::reset() {
    ram_enable     = 0;
    rom_bank_lower = 1;
    rom_bank_upper = 0;
    ram_bank       = 0;
}

// no MBC
noMBC::noMBC(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram) : ROM(rom), SRAM(sram) {}

uint8_t noMBC::readROM(uint16_t addr) {
    uint16_t rom_address = addr;
    return ROM.at(rom_address);
}

void noMBC::writeROM(uint8_t byte, uint16_t addr) { return; }

uint8_t noMBC::readRAM(uint16_t addr) {
    int sram_address = (addr - 0xA000);
    return (SRAM.at(sram_address));
}

void noMBC::writeRAM(uint8_t byte, uint16_t addr) {
    int sram_address      = (addr - 0xA000);
    SRAM.at(sram_address) = byte;
}

void noMBC::reset() {}

// IO Linking
void mmu::linkSERIAL(serial* serial_) { SERIAL = serial_; }

void mmu::linkTIMER(timer* timer_) { TIMER = timer_; }

void mmu::linkPPU(ppu* ppu_) { PPU = ppu_; }

// DMA Action
void mmu::setDMAFlag(){
    dmaFlag = true;
}

void mmu::clearDMAFlag(){
    dmaFlag = false;
}

// Boot ROM 
uint8_t mmu::readBootROM(uint16_t index){
    return bootROM.at(index);
}

void mmu::loadBootROM(){
    std::ifstream rom("boot_rom/dmg_boot.bin", std::ios::binary);
    if (!rom) {
        throw std::runtime_error("Failed to open Boot ROM");
    }
    bootROM = std::vector<uint8_t>((std::istreambuf_iterator<char>(rom)), {});  // Boot ROM vector
}
