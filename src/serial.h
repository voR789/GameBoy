#ifndef SERIAL_H
#define SERIAL_H
#include <cstdint>
#include <iostream>
#include "mmu.h"
// Serial IO Register Class, since simple keep as header only
# pragma once

class serial{
    private:
        mmu& MMU;
        uint8_t SB; // Serial Transfer Data
        uint8_t SC; // Serial Transfer Control
    public:
        serial(mmu& mmu_ref);
        uint8_t readSB();
        uint8_t readSC(); 
        void writeSB(uint8_t byte);
        void writeSC(uint8_t byte);
        void setInterrupt();
};
#endif