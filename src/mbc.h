#include <vector>
#include <cstdint>

// Abstract MBC class for multiple implementations
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
    public:
        MBC5(const std::vector<uint8_t>& rom, std::vector<uint8_t>& sram);

        uint8_t readROM(uint16_t addr) override;
        void writeROM(uint8_t byte, uint16_t addr) override;
        uint8_t readRAM(uint16_t addr) override;
        void writeRAM(uint8_t byte, uint16_t addr) override;
        void reset() override;
};
