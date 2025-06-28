#ifndef MMU_H
#define MMU_H
#include <cstdint>
#include <string>
class mmu{
    private:
        uint8_t memory[65536];
        uint8_t fakeDIV;
    public:
        void clearMem();
        uint8_t readMem(int index);
        void writeMem(uint8_t byte, int index);       
        void loadGame(const std::string& filename);
};

#endif