#ifndef MMU_H
#define MMU_H
#include <cstdint>
#include <string>
class mmu{
    private:
        uint8_t memory[65536];
        uint16_t divCounter;
    public:
        mmu();
        void clearMem();
        void addDivCounter(int cycles);
        uint16_t getDivCounter();
        uint8_t readMem(int index);
        void writeMem(uint8_t byte, int address);       
        void loadGame(const std::string& filename);
};

#endif