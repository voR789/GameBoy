#ifndef MMU_H
#define MMU_H
#include <cstdint>
#include <string>
class mmu{
    private:
        uint8_t memory[65536];
        uint16_t div_counter;
    public:
        mmu();
        void clearMem();
        void addDiv_Counter(int cycles);
        uint16_t getDiv_Counter();
        uint8_t readMem(int index);
        void writeMem(uint8_t byte, int address);       
        void loadGame(const std::string& filename);
};

#endif