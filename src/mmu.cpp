#include "mmu.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
// Timer registers are at:
#define REG_DIV 0xFF04
#define REG_TIMA 0xFF05
#define REG_TMA 0xFF06
#define REG_TAC 0xFF07

mmu::mmu() : divCounter(0) { clearMem(); }

void mmu::clearMem() {
    memset(ROM_0, 0, sizeof(ROM_0));
    memset(SWITCH_ROM, 0, sizeof(SWITCH_ROM));
    memset(VRAM, 0, sizeof(VRAM));
    memset(ERAM, 0, sizeof(ERAM));
    memset(WRAM, 0, sizeof(WRAM));
    memset(ECHO_RAM, 0, sizeof(ECHO_RAM));
    memset(OAM, 0, sizeof(OAM));
    memset(HRAM, 0, sizeof(HRAM));
    IE = 0;
}

// DIV logic
void mmu::addDivCounter(int cycles) { divCounter += cycles; }

uint16_t mmu::getDivCounter() { return divCounter; }

uint8_t mmu::readMem(uint16_t index) {
    if (index <= 0x7FFF) {
        return handleROMRead(index);
    }

    if (index <= 0x9FFF) {
        return VRAM[index - 0x8000];  // Video RAM
    }

    if (index <= 0xBFFF) {
        // MBC RAM
        if (MBC_REG[0]) {
            return handleRAMRead(index);
        } else {
            return 0xFF;
        }
    }

    if (index <= 0xDFFF) {
        return WRAM[index - 0xC000];  // Work RAM
    }

    if (index <= 0xFDFF) {
        return WRAM[index - 0xE000];  // Echo RAM - Mirrors WRAM
    }

    if (index <= 0xFE9F) {
        return OAM[index - 0xFE00];  // Object Attribute Memory
    }

    if (index <= 0xFEFF) {
        return 0;
        // throw std::runtime_error("Read to unusable memory 0xFEA0 - 0xFFEF"); // Unusable Memory
    }

    if (index <= 0xFF7F) {
        switch (index) {  // I/O Registers
            case 0xFF00:
                return 0xCF;  // JOYP (no buttons pressed)
            case 0xFF01:
                return IO_REGISTERS[index - 0xFF00];  // SB (Serial Transfer Data)
            case 0xFF02:
                return IO_REGISTERS[index - 0xFF00];  // SC (Serial Transfer Control)
            case 0xFF04:
                return divCounter >> 8;  // DIV (Counts at 16384 Hz)
            case 0xFF05:
                return IO_REGISTERS[index - 0xFF00];  // TIMA (Timer)
            case 0xFF06:
                return IO_REGISTERS[index - 0xFF00];  // TMA (Timer reset value)
            case 0xFF07:
                return IO_REGISTERS[index - 0xFF00];  // TAC (Controls Timer)
            case 0xFF0F:
                return IO_REGISTERS[index - 0xFF00];  // IF (Interrupt Flag)
            // cases 0xFF10 - FF3F -> audio (not handled)
            case 0xFF40:
                return 0x91;  // LCD Control
            case 0xFF41:
                return 0x85;  // STAT
            case 0xFF44:
                return 0x00;  // LY
            case 0xFF47:
                return 0xFC;  // BGP
            case 0xFF50:
                return IO_REGISTERS[index - 0xFF00];  // Boot ROM mapping control
            default:
                return IO_REGISTERS[index - 0xFF00];  // Stubbed are the CGB registers
        }                                             // TODO: LCD, Boot ROM Mapping
    }

    if (index <= 0xFFFE) {
        return HRAM[index - 0xFF80];
    }

    if (index == 0xFFFF) {
        return IE;
    }

    return 0;
}

void mmu::writeMem(uint8_t byte, uint16_t index) {
    if (index <= 0x7FFF) {
        // writes to this area are not allowed traditionally, instead used as writes to MBC registers
        write_MBC_REG(byte, index);
    }

    if (index <= 0x9FFF) {
        VRAM[index - 0x8000] = byte;
    }

    if (index <= 0xBFFF) {
        if (MBC_REG[0]) {
            handleRAMWrite(byte, index);
        }
    }

    if (index <= 0xDFFF) {
        WRAM[index - 0xC000] = byte;
    }

    if (index <= 0xFDFF) {
        WRAM[index - 0xE000] = byte;  // Write to mirror
    }

    if (index <= 0xFE9F) {
        OAM[index - 0xFE00] = byte;
    }

    if (index <= 0xFEFF) {
        return;
        // throw std::runtime_error("Write to unusable memory 0xFEA0 - 0xFFEF"); // Unusable Memory
    }

    if (index <= 0xFF7F) {
        switch (index) {  // I/O Registers
            case 0xFF01:  // SB (Serial Output Data)
                IO_REGISTERS[index - 0xFF00] = byte;
                break;
            case 0xFF02:  // SC (Serial Output Enable) - Actual wire functionality stubbed
                IO_REGISTERS[index - 0xFF00] = byte;
                if (byte == 0x81) {
                    std::cout << "" << static_cast<char>(IO_REGISTERS[0xFF01 - 0xFF00]);
                    std::cout.flush();
                }
                break;
            case 0xFF04:
                divCounter = 0;  // Represents DIV or 0xFF04
                break;
            case 0xFF05:
                IO_REGISTERS[index - 0xFF00] = byte;  // TIMA
                break;
            case 0xFF06:
                IO_REGISTERS[index - 0xFF00] = byte;  // TMA (Timer Overflow Value)
                break;
            case 0xFF07:
                IO_REGISTERS[index - 0xFF00] =
                    (byte & 0x07);  // TAC (Selects timer speed off bits) -> mask out unused bits
                break;
            case 0xFF0F:
                IO_REGISTERS[index - 0xFF00] = byte;
                // std::cout << "[MMU] IF written: 0x" << std::hex << (int)byte << "\n";
                break;
            default:
                IO_REGISTERS[index - 0xFF00] = byte;  // Stubbed audio,
                break;
        }
        return;
    }  // TODO: LCD, Boot ROM Mapping

    if (index <= 0xFFFE) {
        HRAM[index - 0xFF80] = byte;
    }

    if (index == 0xFFFF) {
        IE = byte;  // mask out unused bytes
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
    romSize  = ROM.at(0x0148);
    ramSize  = ROM.at(0x149);

    MBC_REG[0] = 0x0;
    MBC_REG[1] = 0x1;
    MBC_REG[2] = 0x0;
    MBC_REG[3] = 0x0;
    BANK       = 0x1;

    largeBankMode = false;

    if (cartType <= 0x3) {
        switch (romSize) {
            case (0x00):
                mask = 0x1;
                break;  // only 1 bit needed for 32 Kib cart, etc...
            case (0x01):
                mask = 0x3;
                break;
            case (0x02):
                mask = 0x7;
                break;
            case (0x03):
                mask = 0x0F;
                break;
            case (0x04):
                mask = 0x1F;
                break;  // 5 bit normal pull for 512 Kib and larger
            default: {  // Larger cart (2 additional bit) logic AND MODE 0
                mask          = 0x1F;
                largeBankMode = true;
            }
        }

        switch (ramSize) {
            case (0x0): {
                SRAM = std::vector<uint8_t>(0);
                break;
            }
            case (0x1): {
                SRAM = std::vector<uint8_t>(0);
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
    } else if (cartType <= 0x6) {
        mask = 0xF;
        SRAM = std::vector<uint8_t>(512);
    } else if (cartType <= 0x9) {
        mask = 0xFF;  // stub val
        SRAM = std::vector<uint8_t>(8192);
    } else if (cartType <= 0x13) {  // MBC 3
        mask = 0x7F;
        switch (ramSize) {
            case (0x0): {
                SRAM = std::vector<uint8_t>(0);
                break;
            }
            case (0x1): {
                SRAM = std::vector<uint8_t>(0);
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
    } else if (cartType <= 0x1E){ // MBC5
        mask = 0x1FF;
        switch (ramSize) {
            case (0x0): {
                SRAM = std::vector<uint8_t>(0);
                break;
            }
            case (0x1): {
                SRAM = std::vector<uint8_t>(0);
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
        }
    }
}

uint8_t mmu::handleROMRead(uint16_t index) {
    int address;                    // Convert from index into ROM address, based on MBC
    if (cartType <= 0x3) {          // MBC1
        if (MBC_REG[3] == 0) {      // banking mode 0
            if (index <= 0x3FFF) {  // Lower Bank
                address = index;
            } else {                         // Upper Bank
                address = (BANK * 0x4000) |  // 16 Kib banks (0x4000)
                          (index - 0x4000);  // Every bank is individual, so we subtract 0x4000 to act at 0...
            }
        } else {  // banking mode 1
            if (index <= 0x3FFF) {
                address = (MBC_REG[2] << 19) | index;
            } else {
                address = (BANK * 0x4000) | (index - 0x4000);
            }
        }
    } else if (cartType <= 0x6) {  // MBC2
        if (index <= 0x3FFF) {
            address = index;
        } else {
            address = (BANK * 0x4000) | (index - 0x4000);
        }
    } else if (cartType <= 0x9) {
        address = index;
    } else if (cartType <= 0x0D) {
        // Incomplete MMM01 - multigame compilation mode -> Stubbed as MBC1
        if (MBC_REG[3] == 0) {     
            if (index <= 0x3FFF) {  
                address = index;
            } else {                       
                address = (BANK * 0x4000) | (index - 0x4000);  
            }
        } else {  // banking mode 1
            if (index <= 0x3FFF) {
                address = (MBC_REG[2] << 19) | index;
            } else {
                address = (BANK * 0x4000) | (index - 0x4000);
            }
        }
    } else if (cartType <= 0x13) {  // MBC 3
        if (index <= 0x3FFF) {
            address = index;
        } else {
            address = (BANK * 0x4000) | (index - 0x4000);
        }
    } else if (cartType <= 0x1E){ // MBC5
        if(address <= 0x3FFF){
            address = index;
        } else {
            address = (BANK * 0x4000) | (index - 0x4000);
        }
    }
    return ROM.at(address);
}

uint8_t mmu::handleRAMRead(uint16_t index) {
    int address;
    if (cartType <= 0x3) {
        if (MBC_REG[3] == 0) {  // Banking mode 0
            address = index - 0xA000;
        } else {                                                 // Banking mode 1
            address = (MBC_REG[2] * 0x2000) | (index - 0xA000);  // 8 Kib Banks
        }
    } else if (cartType <= 0x6) {
        if ((index >= 0xA000) && (index <= 0xA1FF)) {
            address = index - 0xA000;
        } else {  // Echo behavior
            address = (index - 0xA000) % 512;
        }
    } else if (cartType <= 0x9) {
        address = index - 0xA000;
    } else if (cartType <= 0x0D) {
        // Incomplete MMM01 - multigame compilation mode
        if (MBC_REG[3] == 0) {  
            address = index - 0xA000;
        } else {                                                
            address = (MBC_REG[2] * 0x2000) | (index - 0xA000); 
        }
    } else if (cartType <= 0x13) {  // MBC 3
        if (MBC_REG[2] <= 0x03) {  // RAM Bank
            address = (MBC_REG[2] * 0x2000) | (index - 0xA000);
        } else if (MBC_REG[2] >= 0x08 && MBC_REG[2] <= 0x0C) {  // RTC
            // Stubbed
            return 0x0;
        } 
    } else if (cartType <= 0x1E){ // MBC5
        address = (MBC_REG[2] * 0x2000) | (index - 0xA000);
    }
    return SRAM.at(address);
}

// Add in battery and save functionality
void mmu::handleRAMWrite(uint8_t byte, uint16_t index) {
    int address;
    if (cartType <= 0x3) {
        if (MBC_REG[3] == 0) {  // Banking mode 0
            address = index - 0xA000;
        } else {                                                 // Banking mode 1
            address = (MBC_REG[2] * 0x2000) | (index - 0xA000);  // 8 Kib Banks
        }
    } else if (cartType <= 0x6) {
        if ((index >= 0xA000) && (index <= 0xA1FF)) {
            address = index - 0xA000;
        } else {  // Echo behavior
            address = (index - 0xA000) % 512;
        }
    } else if (cartType <= 0x9) {
        address = index - 0xA000;
    } else if (cartType <= 0x0D) {
        if (MBC_REG[3] == 0) {  
            address = index - 0xA000;
        } else {                                                 
            address = (MBC_REG[2] * 0x2000) | (index - 0xA000); 
        }
    } else if (cartType <= 0x13) {  // MBC 3
        if (MBC_REG[2] <= 0x03) {
            address          = (MBC_REG[2] * 0x2000) | (index - 0xA000);
            SRAM.at(address) = byte;
        } else if (MBC_REG[2] >= 0x08 && MBC_REG[2] <= 0x0C) {
            // RTC register write stub
        }
    } else if (cartType <= 0x1E){ // MBC5
        address = (MBC_REG[2] * 0x2000) | (index - 0xA000);
    }

    SRAM.at(address) = byte;
}

void mmu::write_MBC_REG(uint8_t byte, uint16_t address) {
    if (cartType <= 0x3) {  // MBC1 Behavior
        if (0x0000 <= address and address <= 0x1FFF) {
            // RAM Enable
            if ((byte & 0xF) == 0xA) {
                MBC_REG[0] = 0x1;  // Enabled
            } else {
                MBC_REG[0] = 0x0;  // Disabled
            }
        }

        if (0x2000 <= address and address <= 0x3FFF) {
            // ROM Bank
            MBC_REG[1] = byte;
            if ((MBC_REG[1] & 0x1F) == 0x0) {
                MBC_REG[1] |= 0x1;
            }  // 0x0 Conversion

            if (largeBankMode) {
                BANK = (MBC_REG[1] & mask) + (MBC_REG[2] << 5);  // make occupy bits 5/6 for addition
            } else {
                BANK = MBC_REG[1] & mask;
            }
        }

        if (0x4000 <= address and address <= 0x5FFF) {
            // RAM Bank Number, or Upper ROM Bank Number
            MBC_REG[2] = byte & 0x3;
        }

        if (0x6000 <= address and address <= 0x7FFF) {
            // Banking Mode Select
            MBC_REG[3] = byte & 0x1;
        }
    } else if (cartType <= 0x6) {  // MBC2 Behavior
        if (address <= 0x3FFF) {
            if ((address >> 8) & 0x1) {  // Bit 8 set
                MBC_REG[1] = byte;
                if ((MBC_REG[1] & 0xF) == 0x0) {
                    MBC_REG[1] == 0x1;
                }
                BANK = MBC_REG[1] & mask;

            } else {  // Bit 8 Clear
                if ((byte & 0xF) == 0xA) {
                    MBC_REG[0] = 0x1;  // RAM Enable
                } else {
                    MBC_REG[0] = 0x0;  // RAM Disable
                }
            }
        } else {
            throw std::runtime_error("Write to illegal address: MBC2");
        }
    } else if (cartType <= 0x0D) {
        if (0x0000 <= address and address <= 0x1FFF) {
            if ((byte & 0xF) == 0xA) {
                MBC_REG[0] = 0x1;  
            } else {
                MBC_REG[0] = 0x0;  
            }
        }

        if (0x2000 <= address and address <= 0x3FFF) {
            // ROM Bank
            MBC_REG[1] = byte;
            if ((MBC_REG[1] & 0x1F) == 0x0) {
                MBC_REG[1] |= 0x1;
            }

            if (largeBankMode) {
                BANK = (MBC_REG[1] & mask) + (MBC_REG[2] << 5); 
            } else {
                BANK = MBC_REG[1] & mask;
            }
        }

        if (0x4000 <= address and address <= 0x5FFF) {
            // RAM Bank Number, or Upper ROM Bank Number
            MBC_REG[2] = byte & 0x3;
        }

        if (0x6000 <= address and address <= 0x7FFF) {
            // Banking Mode Select
            MBC_REG[3] = byte & 0x1;
        }
    } else if (cartType <= 0x13) {  // MBC 3
        if (0x0000 <= address and address <= 0x1FFF) { // RAM Enable
            if ((byte & 0xF) == 0xA) {
                MBC_REG[0] = 0x1;  // Enabled
            } else {
                MBC_REG[0] = 0x0;  // Disabled
            }
        }
        if (0x2000 <= address and address <= 0x3FFF) {
            MBC_REG[1] = byte;
            if (MBC_REG[1] == 0x0) {
                MBC_REG[1] |= 0x1;
            }
            BANK = MBC_REG[1] & mask;
        }
        if (0x4000 <= address and address <= 0x5FFF) {
            MBC_REG[2] = byte;  // Controls RAM Bank or RTC Register
        }
        if (0x6000 <= address and address <= 0x7FFF) {
            // Stubbed
        }
        if (0xA000 <= address and address <= 0xBFFF) {
            // Stubbed
        }
    } else if (cartType <= 0x1E){ // MBC5
        if (0x0000 <= address and address <= 0x1FFF) { // RAM Enable
            if ((byte & 0xF) == 0xA) {
                MBC_REG[0] = 0x1;  // Enabled
            } else {
                MBC_REG[0] = 0x0;  // Disabled
            }
        }
        if (0x2000 <= address and address <= 0x3FFF) {
            BANK = byte & mask;
        }
        if (0x4000 <= address and address <= 0x5FFF) {
            BANK = (BANK | (byte << 8)) & mask;
        }
        if (0x6000 <= address and address <= 0x7FFF) {
            if(byte <= 0x0F){
                MBC_REG[2] = byte;
            }
        }
    }
}
