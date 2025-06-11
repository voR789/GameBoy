#ifndef MMU_H
#define MMU_H
class mmu{
    private:
        uint8_t memory[65536];
    public:
        uint8_t readMem(int index);
        void writeMem(uint8_t byte, int index);
};

#endif