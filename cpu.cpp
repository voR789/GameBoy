#include <string>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <GL/glut.h>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"
#include "timer.h"

using std::string, std::memset, std::ifstream, std::cerr;

// Helper functions
#pragma region
uint8_t cpu::getUpper(uint16_t data)
{
    return ((data & 0xFF00) >> 8);
}

uint8_t cpu::getLower(uint16_t data)
{
    return (data & 0xFF);
}

void cpu::clearAllFlags()
{
    uint16_t AF = registers[0];
    uint8_t A = getUpper(AF);
    registers[0] = A << 8;
}

bool cpu::getFlag(char flag)
{
    uint8_t F = getLower(registers[4]);
    switch (flag)
    {
    case ('Z'):
        return (F & Z_FLAG) != 0;
    case ('N'):
        return (F & Z_FLAG) != 0;
    case ('H'):
        return (F & Z_FLAG) != 0;
    case ('C'):
        return (F & Z_FLAG) != 0;
    default:
        throw std::runtime_error("getFlag(): Invalid Flag Call");
    }
}

void cpu::setFlag(char flag)
{
    uint8_t F = getLower(registers[4]);
    switch (flag)
    {
    case ('Z'):
        F |= Z_FLAG;
        break;
    case ('N'):
        F |= N_FLAG;
        break;
    case ('H'):
        F |= H_FLAG;
        break;
    case ('C'):
        F |= C_FLAG;
        break;
    default:
        throw std::runtime_error("setFlag(): Invalid Flag Call");
    }
    // Pushing changes to F register
    registers[4] |= F;
}

void cpu::clearFlag(char flag)
{
    uint8_t F = getLower(registers[4]);
    switch (flag)
    {
    case ('Z'):
        F &= ~Z_FLAG;
        break;
    case ('N'):
        F &= ~N_FLAG;
        break;
    case ('H'):
        F &= ~H_FLAG;
        break;
    case ('C'):
        F &= ~C_FLAG;
        break;
    default:
        throw std::runtime_error("clearFlag(): Invalid Flag Call");
    }
    // Pushing changes to F register
    registers[4] &= F;
}

void cpu::fetchOpcode()
{
    opcode = MMU.readMem(pc++);
}

uint8_t cpu::fetchNextByte()
{
    return MMU.readMem(pc++);
}

uint16_t cpu::fetchNext2Bytes()
{
    return (MMU.readMem(pc++) | (MMU.readMem(pc++) << 8));
}

// TODO: rewrite inc8 and dec8 to be all encompassing
void cpu::inc8(char reg)
{
    uint8_t byte;
    switch (reg)
    {
    case ('A'):
        byte = getUpper(registers[0]);
        uint8_t F = getLower(registers[0]);

        // half carry
        if ((byte & 0xF) + 1 > 0xF)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        }
        // dont touch carry flag

        registers[0] = ((byte + 1) << 8) | F;
        break;
        // no "F" - reserved for flags
    case ('B'):
        byte = getUpper(registers[1]);
        uint8_t C = getLower(registers[1]);

        if ((byte & 0xF) + 1 > 0xF)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        }

        registers[1] = ((byte + 1) << 8) | C;
        break;
    case ('C'):
        byte = getLower(registers[1]);
        uint8_t B = getUpper(registers[1]);

        if ((byte & 0xF) + 1 > 0xF)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        }

        registers[1] = (B << 8) | (byte + 1);
        break;

    case ('D'):
        byte = getUpper(registers[2]);
        uint8_t E = getLower(registers[2]);

        if ((byte & 0xF) + 1 > 0xF)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        }

        registers[2] = ((byte + 1) << 8) | E;
        break;
    case ('E'):
        byte = getLower(registers[2]);
        uint8_t D = getUpper(registers[2]);

        if ((byte & 0xF) + 1 > 0xF)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        }

        registers[2] = (D << 8) | (byte + 1);
        break;
    case ('H'):
        byte = getUpper(registers[3]);
        uint8_t L = getLower(registers[3]);

        if ((byte & 0xF) + 1 > 0xF)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        }

        registers[3] = ((byte + 1) << 8) | L;
        break;
    case ('L'):
        byte = getLower(registers[3]);
        uint8_t H = getUpper(registers[3]);

        if ((byte & 0xF) + 1 > 0xF)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        }

        registers[3] = (H << 8) | (byte + 1);
        break;
    default:
        throw std::runtime_error("ldReg8 Error, no case");
    }

    if ((uint8_t)(byte + 1) == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
}

void cpu::dec8(char reg)
{
    uint8_t byte;
    switch (reg)
    {
    case ('A'):
        byte = getUpper(registers[0]);
        uint8_t F = getLower(registers[0]);

        // carry flags indicate borrows
        if ((byte & 0xF) == 0)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        } // if last nibble is zero, then it will borrow

        registers[0] = ((byte - 1) << 8) | F;
        break;
        // no "F" - reserved for flags
    case ('B'):
        byte = getUpper(registers[1]);
        uint8_t C = getLower(registers[1]);

        if ((byte & 0xF) == 0)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        } // if last nibble is zero, then it will borrow

        registers[1] = ((byte - 1) << 8) | C;
        break;
    case ('C'):
        byte = getLower(registers[1]);
        uint8_t B = getUpper(registers[1]);

        if ((byte & 0xF) == 0)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        } // if last nibble is zero, then it will borrow

        registers[1] = (B << 8) | (byte - 1);
        break;

    case ('D'):
        byte = getUpper(registers[2]);
        uint8_t E = getLower(registers[2]);

        if ((byte & 0xF) == 0)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        } // if last nibble is zero, then it will borrow

        registers[2] = ((byte - 1) << 8) | E;
        break;
    case ('E'):
        byte = getLower(registers[2]);
        uint8_t D = getUpper(registers[2]);

        if ((byte & 0xF) == 0)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        } // if last nibble is zero, then it will borrow

        registers[2] = (D << 8) | (byte - 1);
        break;
    case ('H'):
        byte = getUpper(registers[3]);
        uint8_t L = getLower(registers[3]);

        if ((byte & 0xF) == 0)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        } // if last nibble is zero, then it will borrow

        registers[3] = ((byte - 1) << 8) | L;
        break;
    case ('L'):
        byte = getLower(registers[3]);
        uint8_t H = getUpper(registers[3]);

        if ((byte & 0xF) == 0)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        } // if last nibble is zero, then it will borrow

        registers[3] = (H << 8) | (byte - 1);
        break;
    default:
        throw std::runtime_error("ldReg8 Error, no case");
    }

    if ((uint8_t)(byte - 1) == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
}
/* no flags set when inc/dec 16 bit
void cpu::inc16(uint16_t bytes){

}


void cpu::dec16(uint16_t bytes){

}*/
// TODO: Accumulator arithmetic functions and bitwise
void cpu::addA(uint8_t byte){};
void cpu::acdA(uint8_t byte){};
void cpu::subA(uint8_t byte){};
void cpu::sbcA(uint8_t byte){};

uint16_t cpu::add16(uint16_t byte1, uint16_t byte2)
{
    clearFlag('N');
    if ((byte1 & 0x0FFF) + (byte2 & 0x0FFF) > 0x0FFF)
    {
        setFlag('H');
    }
    else
    {
        clearFlag('H');
    }
    if ((byte1 + byte2) > 0xFFFF)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }
    return byte1+byte2;
}

void cpu::ldMem8(int address, uint8_t byte)
{
    try
    {
        MMU.writeMem(byte, address);
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("ldMem8 Error");
    }
}

void cpu::ldReg8(char reg, uint8_t byte)
{
    switch (reg)
    {
    case ('A'):
    {
        uint8_t F = getLower(registers[0]);
        registers[0] = (byte << 8) | F;
        break;
    } // no "F" - reserved for flags
    case ('B'):
    {
        uint8_t C = getLower(registers[1]);
        registers[1] = (byte << 8) | C;
        break;
    }
    case ('C'):
    {
        uint8_t B = getUpper(registers[1]);
        registers[1] = (B << 8) | byte;
        break;
    }
    case ('D'):
    {
        uint8_t E = getLower(registers[2]);
        registers[2] = (byte << 8) | E;
        break;
    }
    case ('E'):
    {
        uint8_t D = getUpper(registers[2]);
        registers[2] = (D << 8) | byte;
        break;
    }
    case ('H'):
    {
        uint8_t L = getLower(registers[3]);
        registers[3] = (byte << 8) | L;
        break;
    }
    case ('L'):
    {
        uint8_t H = getUpper(registers[3]);
        registers[3] = (H << 8) | byte;
        break;
    }
    default:
        throw std::runtime_error("ldReg8 Error, no case");
    }
}

int cpu::execute()
{
    // switch based off of default table
    // return cycles
    uint8_t firstNibble = (opcode & 0xF0) >> 4;
    uint8_t secondNibble = opcode & 0x0F;
    switch (firstNibble)
    {
    case (0x0):
    {
        switch (secondNibble)
        {
        case (0x0):
            // do nothing
            return 1;

        case (0x1):
            // LD BC, d16
            registers[1] = fetchNext2Bytes();
            return 3;

        case (0x2):
            uint8_t A = getUpper(registers[0]);
            int BC = registers[1];
            ldMem8(BC, A);
            return 2;

        case (0x3):
            registers[1]++;
            // no flag checks for 16 bit inc/dec
            return 2;

        case (0x4):
            inc8('B');
            return 1;

        case (0x5):
            dec8('B');
            return 1;

        case (0x6):
            ldReg8('B', fetchNextByte());
            return 2;

        case (0x7):
            uint8_t A = getUpper(registers[0]);
            uint8_t F = getLower(registers[0]);
            bool leftMost = A >> 7;
            if (leftMost)
            {
                setFlag('C');
            }
            else
            {
                clearFlag('C');
            }
            A = (A << 1) | leftMost;
            registers[0] = (A << 8) | F;
            return 1;

        case (0x8):
            uint8_t lower = getLower(sp);
            uint8_t upper = getUpper(sp);
            uint16_t location = fetchNext2Bytes();
            MMU.writeMem(lower, location);
            MMU.writeMem(upper, location + 1);
            return 5;

        case (0x9):
            uint16_t BC = registers[1];
            uint16_t HL = registers[3];

            // set flags
            registers[3] = add16(BC, HL);

            return 2;

        case (0xA):
            uint16_t BC_data = MMU.readMem(registers[1]);
            ldReg8('A', BC_data);

            return 2;

        case (0xB):
            registers[1]--;

            return 2;

        case (0xC):
            inc8('C');
            return 1;

        case (0xD):
            dec8('C');
            return 1;

        case (0xE):
            uint8_t B = getUpper(registers[1]);
            uint8_t C = getLower(registers[1]);
            C = fetchNextByte();

            registers[1] = (B << 8) | C;
            return 2;

        case (0xF):
            uint8_t A = getUpper(registers[0]);
            uint8_t F = getLower(registers[0]);
            bool rightMost = A & 0x1;
            if (rightMost)
            {
                setFlag('C');
            }
            else
            {
                clearFlag('C');
            }
            A = (A >> 1) | (rightMost << 7);
            registers[0] = (A << 8) | F;

            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x1):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO: do after figuring out interrupt flags*/

        case (0x1):
            registers[2] = fetchNext2Bytes();
            return 3;

        case (0x2):
            uint8_t A = getUpper(registers[0]);
            int DE = registers[2];
            ldMem8(DE, A);
            return 2;

        case (0x3):
            registers[2]++;
            // no flag checks for 16 bit inc/dec
            return 2;

        case (0x4):
            inc8('D');
            return 1;

        case (0x5):
            dec8('D');
            return 1;

        case (0x6):
            ldReg8('D', fetchNextByte());
            return 2;

        case (0x7):
            uint8_t A = getUpper(registers[0]);
            uint8_t F = getLower(registers[0]);
            bool leftMost = A >> 7;
            A = (A << 1) | (getFlag('C'));
            if (leftMost)
            {
                setFlag('C');
            }
            else
            {
                clearFlag('C');
            }
            return 1;

        case (0x8):
            pc += fetchNextByte();
            return 3;

        case (0x9):
            uint16_t DE = registers[2];
            uint16_t HL = registers[3];
            registers[3] = add16(DE, HL);
            // set flags

            return 2;

        case (0xA):
            uint16_t DE_data = MMU.readMem(registers[2]);
            ldReg8('A', DE_data);

            return 2;

        case (0xB):
            registers[2]--;

            return 2;

        case (0xC):
            inc8('E');
            return 1;

        case (0xD):
            dec8('E');
            return 1;

        case (0xE):
            uint8_t D = getUpper(registers[2]);
            uint8_t E = getLower(registers[2]);
            E = fetchNextByte();

            registers[1] = (D << 8) | E;
            return 2;

        case (0xF):
            uint8_t A = getUpper(registers[0]);
            uint8_t F = getLower(registers[0]);
            bool rightMost = A & 0x1;
            if (rightMost)
            {
                setFlag('C');
            }
            else
            {
                clearFlag('C');
            }
            A = (A >> 1) | (getFlag('C') << 7);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x2):
    {
        switch (secondNibble)
        {
        case (0x0):
            if (getFlag('Z') == 0)
            {
                pc += fetchNextByte();
                return 3;
            }
            else
            {
                fetchNextByte();
                return 2;
            }

        case (0x1):
            registers[3] = fetchNext2Bytes();
            return 3;

        case (0x2):
            uint8_t A = getUpper(registers[0]);
            int HL = registers[3];
            ldMem8(HL++, A);
            registers[3] = HL;
            return 2;

        case (0x3):
            registers[3]++;
            return 2;

        case (0x4):
            inc8('H');
            return 1;

        case (0x5):
            dec8('H');
            return 1;

        case (0x6):
            ldReg8('H', fetchNextByte());
            return 2;

        case (0x7):
            // A -> binary coded decimal
            // add base ten * six to turn from Hex to Decimal, and use H/C flags
            uint8_t A = getUpper(registers[0]);
            uint8_t F = getLower(registers[0]);
            uint8_t lower_nibble = A & 0xF;
            uint8_t upper_nibble = A & 0xF0;
            if (lower_nibble > 0x9 || getFlag('H'))
            {
                A += 0x6;
            }
            clearFlag('H'); // clear, as pretty much only used for BCD
            if (upper_nibble > 0x90 || getFlag('C'))
            {
                A += 0x60;
                setFlag('C');
            }

            if (A == 0)
            {
                setFlag('Z');
            }
            else
            {
                clearFlag('Z');
            }

            registers[0] = (A << 8) | F;
            return 1;

        case (0x8):
            if (getFlag('Z'))
            {
                pc += fetchNextByte();
                return 3;
            }
            else
            {
                fetchNextByte();
                return 2;
            }

        case (0x9):
            uint16_t HL = registers[3];
            registers[3] = add16(HL, HL);
            return 2;

        case (0xA):
            uint16_t HL = registers[3];
            ldReg8('A', HL);
            HL++;
            return 2;

        case (0xB):
            registers[3]--;
            return 2;

        case (0xC):
            inc8('L');
            return 1;

        case (0xD):
            dec8('H');
            return 1;

        case (0xE):
            ldReg8('E', fetchNextByte());
            return 2;

        case (0xF):
            uint8_t A = ~getUpper(registers[0]);
            uint8_t F = getLower(registers[0]);
            registers[0] = (A << 8) | F;
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x3):
    {
        switch (secondNibble)
        {
        case (0x0):
            if (getFlag('C') == 0)
            {
                pc += fetchNextByte();
                return 3;
            }
            else
            {
                fetchNextByte();
                return 2;
            }

        case (0x1):
            sp = fetchNext2Bytes();
            return 3;

        case (0x2):
            uint8_t A = getUpper(registers[0]);
            uint16_t HL = registers[3];
            ldMem8(HL--,A);
            registers[3] = HL;
            return 2;

        case (0x3):
            sp++;
            return 2;

        case (0x4):
            registers[3]++;
            return 2;

        case (0x5):
            registers[3]--;
            return 2;

        case (0x6):
            ldMem8(registers[3], fetchNextByte());

        case (0x7):
            setFlag('C');

        case (0x8):
            if(getFlag('C')){
                pc += fetchNextByte();
                return 3;
            } else{
                fetchNextByte();
                return 2;
            }

        case (0x9):
            uint16_t HL = registers[3];
            registers[3] = add16(HL,sp);
            return 2;

        case (0xA):
            uint8_t A = getUpper(registers[0]);
            uint16_t HL = registers[3];
            ldMem8(HL--,A);
            registers[3] = HL;            
            return 2;

        case (0xB):
            sp--;
            return 2;

        case (0xC):
            inc8('A');
            return 1;

        case (0xD):
            dec8('A');
            return 1;

        case (0xE):
            ldReg8('A',fetchNextByte());
            return 2;

        case (0xF):
            if(getFlag('C')) {clearFlag('C');} else {setFlag('C');}
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x4):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B',B);
            return 1;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('B',C);
            return 1;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('B',D);
            return 1;            
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('B',E);
            return 1;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('B',H);
            return 1;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('B',L);
            return 1;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('B',HL_data);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('B', A);
            return 1;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('C',B);
            return 1;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C',C);
            return 1;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('C',D);
            return 1;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('C',E);
            return 1;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('C',H);
            return 1;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('C',L);
            return 1;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('C',HL_data);
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('C', A);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x5):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('D',B);
            return 1;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('D',C);
            return 1;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D',D);
            return 1;            
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('D',E);
            return 1;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('D',H);
            return 1;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('D',L);
            return 1;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('D',HL_data);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('D', A);
            return 1;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('E',B);
            return 1;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('E',C);
            return 1;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('E',D);
            return 1;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E',E);
            return 1;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('E',H);
            return 1;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('E',L);
            return 1;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('E',HL_data);
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('E', A);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x6):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('H',B);
            return 1;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('H',C);
            return 1;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('H',D);
            return 1;            
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('H',E);
            return 1;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H',H);
            return 1;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('H',L);
            return 1;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('H',HL_data);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('H', A);
            return 1;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('L',B);
            return 1;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('L',C);
            return 1;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('L',D);
            return 1;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('L',E);
            return 1;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('L',H);
            return 1;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L',L);
            return 1;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('L',HL_data);
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('L', A);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x7):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint16_t HL = registers[3];
            uint8_t B = getUpper(registers[1]);
            ldMem8(HL,B);
            return 2;
        case (0x1):
            uint16_t HL = registers[3];
            uint8_t C = getLower(registers[1]);
            ldMem8(HL,C);
            return 2;
        case (0x2):
            uint16_t HL = registers[3];
            uint8_t D = getUpper(registers[2]);
            ldMem8(HL,D);
            return 2;
        case (0x3):
            uint16_t HL = registers[3];
            uint8_t E = getLower(registers[2]);
            ldMem8(HL,E);
            return 2;
        case (0x4):
            uint16_t HL = registers[3];
            uint8_t H = getUpper(registers[3]);
            ldMem8(HL,H);
            return 2;
        case (0x5):
            uint16_t HL = registers[3];
            uint8_t L = getLower(registers[3]);
            ldMem8(HL,L);
            return 2;
        case (0x6):
            /*TODO: HALT*/

        case (0x7):
            uint16_t HL = registers[3];
            uint8_t A = getUpper(registers[0]);
            ldMem8(HL,A);
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('A',B);
            return 1;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('A',C);
            return 1;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('A',D);
            return 1;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('A',E);
            return 1;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('A',H);
            return 1;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('A',L);
            return 1;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('A',HL_data);
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', A);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x8):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x9):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xA):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xB):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xC):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xD):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xE):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xF):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    default:
        throw std::runtime_error("Invalid Opcode!");
    }
}

int cpu::executePrefixed()
{
    fetchOpcode();
    // switch based off of prefix table
    uint8_t firstNibble = (opcode & 0xF0) >> 4;
    uint8_t secondNibble = opcode & 0x0F;
    switch (firstNibble)
    {
    case (0x0):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x1):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x2):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x3):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x4):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x5):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x6):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x7):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x8):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x9):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xA):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xB):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xC):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xD):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xE):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xF):
    {
        switch (secondNibble)
        {
        case (0x0):
            /*TODO:*/

        case (0x1):
            /*TODO:*/

        case (0x2):
            /*TODO:*/

        case (0x3):
            /*TODO:*/

        case (0x4):
            /*TODO:*/

        case (0x5):
            /*TODO:*/

        case (0x6):
            /*TODO:*/

        case (0x7):
            /*TODO:*/

        case (0x8):
            /*TODO:*/

        case (0x9):
            /*TODO:*/

        case (0xA):
            /*TODO:*/

        case (0xB):
            /*TODO:*/

        case (0xC):
            /*TODO:*/

        case (0xD):
            /*TODO:*/

        case (0xE):
            /*TODO:*/

        case (0xF):
            /*TODO:*/

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    default:
        throw std::runtime_error("Invalid Opcode!");
    }
}

int cpu::executeOpcode()
{
    if (opcode == 0xCB)
    {
        return executePrefixed();
    }
    else
    {
        return execute();
    }
}

#pragma endregion

// Constructor
cpu::cpu(mmu &MMUref, ppu &PPUref, timer &TIMERref) : MMU(MMUref), PPU(PPUref), TIMER(TIMERref)
{
    memset(registers, 0, sizeof(registers));
    pc = 0x0100; // Right above boot rom
    sp = 0xFFFE; // only lives in high ram for boot, then moves to work ram, downwardly
    opcode = 0;
}

// Game loop
int cpu::step()
{
    fetchOpcode();
    // return cycles to help sync with ppu and timer
}
