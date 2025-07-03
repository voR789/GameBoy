#ifndef MMU_H
#define MMU_H
#include <cstdint>
#include <string>
class mmu{
    private:
        uint8_t memory[65536];
        bool divReset;
    public:
        mmu();
        void clearMem();
        bool getDivReset();
        void clearDivReset();
        uint8_t readMem(int index);
        void writeMem(uint8_t byte, int address);       
        void loadGame(const std::string& filename);
};

#endif