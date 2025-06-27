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

bool cpu::getFlag(char flag)
{
    uint8_t F = getLower(registers[0]);
    switch (flag)
    {
    case ('Z'):
        return (F & Z_FLAG);
    case ('N'):
        return (F & N_FLAG);
    case ('H'):
        return (F & H_FLAG);
    case ('C'):
        return (F & C_FLAG);
    default:
        throw std::runtime_error("getFlag(): Invalid Flag Call");
    }
}

void cpu::setFlag(char flag)
{
    uint8_t A = getUpper(registers[0]);
    uint8_t F = getLower(registers[0]);
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
    registers[0] = (A << 8) | F;
}

void cpu::clearFlag(char flag)
{
    uint8_t A = getUpper(registers[0]);
    uint8_t F = getLower(registers[0]);
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
    registers[0] = (A << 8) | F;
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
    return (MMU.readMem(pc++) | (MMU.readMem(pc++) << 8)); // little endian read
}

void cpu::inc8(char reg)
{
    uint8_t byte;
    switch (reg)
    {
    case ('A'):
        byte = getUpper(registers[0]);

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

        uint8_t F = getLower(registers[0]);
        registers[0] = ((byte + 1) << 8) | F; // push to AF register
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
        throw std::runtime_error("inc8 Error, no case");
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

        // carry flags indicate borrows
        if ((byte & 0xF) == 0)
        {
            setFlag('H');
        }
        else
        {
            clearFlag('H');
        } // if last nibble is zero, then it will borrow

        uint8_t F = getLower(registers[0]);
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
        throw std::runtime_error("dec8 Error, no case");
    }

    if ((uint8_t)(byte - 1) == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    setFlag('N');
}

uint16_t cpu::add16(uint16_t byte1, uint16_t byte2)
{
    // no zero flag
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
    return byte1 + byte2;
}

void cpu::addA(uint8_t byte)
{
    uint8_t A = getUpper(registers[0]);

    // Z, N, H, and Cy flags
    if ((uint8_t)(A + byte) == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }

    clearFlag('N');

    if ((A & 0xF) + (byte & 0xF) > 0xF)
    {
        setFlag('H');
    }
    else
    {
        clearFlag('H');
    }

    if (A + byte > 0xFF)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }

    A += byte;
    uint8_t F = getLower(registers[0]);
    registers[0] = (A << 8) | F;
}

void cpu::adcA(uint8_t byte)
{
    uint8_t A = getUpper(registers[0]);
    bool Cy = getFlag('C');

    // Z, N, H, and Cy flags
    if ((uint8_t)(A + byte + Cy) == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }

    clearFlag('N');

    if (((A & 0xF) + (byte & 0xF) + Cy) > 0xF)
    {
        setFlag('H');
    }
    else
    {
        clearFlag('H');
    }

    if ((A + byte + Cy) > 0xFF)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }

    A += (byte + Cy);
    uint8_t F = getLower(registers[0]);
    registers[0] = (A << 8) | F;
}

void cpu::subA(uint8_t byte)
{
    uint8_t A = getUpper(registers[0]);

    // Z, N, H, and Cy flags
    if ((uint8_t)(A - byte) == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }

    setFlag('N');

    if ((A & 0xF) < (byte & 0xF))
    {
        setFlag('H');
    }
    else
    {
        clearFlag('H');
    }

    if (A < byte)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }

    A -= byte;
    uint8_t F = getLower(registers[0]);
    registers[0] = (A << 8) | F;
}

void cpu::sbcA(uint8_t byte)
{
    uint8_t A = getUpper(registers[0]);
    bool Cy = getFlag('C');

    // Z, N, H, and Cy flags
    if ((uint8_t)(A - (byte + Cy)) == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }

    setFlag('N');

    if ((A & 0xF) < ((byte & 0xF) + Cy))
    {
        setFlag('H');
    }
    else
    {
        clearFlag('H');
    }

    if (A < (byte + Cy))
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }

    A -= (byte + Cy);
    uint8_t F = getLower(registers[0]);
    registers[0] = (A << 8) | F;
}

void cpu::andA(uint8_t byte)
{
    uint8_t A = getUpper(registers[0]);
    A &= byte;

    // Flag setting
    if (A == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    setFlag('H');
    clearFlag('C');

    uint8_t F = getLower(registers[0]);
    registers[0] = (A << 8) | F;
}

void cpu::xorA(uint8_t byte)
{
    uint8_t A = getUpper(registers[0]);
    A ^= byte;

    // Flag setting
    if (A == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');
    clearFlag('C');
    uint8_t F = getLower(registers[0]);
    registers[0] = (A << 8) | F;
}

void cpu::orA(uint8_t byte)
{
    uint8_t A = getUpper(registers[0]);
    A |= byte;
    if (A == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');
    clearFlag('C');
    uint8_t F = getLower(registers[0]);
    registers[0] = (A << 8) | F;
}

void cpu::cpA(uint8_t byte)
{ // basically subA, but just to set flags
    uint8_t A = getUpper(registers[0]);

    // Z, N, H, and Cy flags
    if ((uint8_t)(A - byte) == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }

    setFlag('N');

    if ((A & 0xF) < (byte & 0xF))
    {
        setFlag('H');
    }
    else
    {
        clearFlag('H');
    }

    if (A < byte)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }
}

uint8_t cpu::popStack()
{
    return MMU.readMem(sp++);
    // the stack in the gb memory map is reverse mapped, starting at 0x..., and decreasing to 0x..., so ++ is "popping" in this context
}

void cpu::pushStack(uint8_t byte)
{
    MMU.writeMem(byte, --sp);
    // pre dec to move stack pointer to new space before writing
}

void cpu::storePC()
{
    uint8_t upper = ((pc & 0xFF00) >> 8);
    uint8_t lower = (pc & 0xFF);
    pushStack(upper);
    pushStack(lower);
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
            ldReg8('A', RLC(A));
            clearFlag('Z'); // edge case with 0 flags
            clearFlag('N');
            clearFlag('H');
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
            ldReg8('C', fetchNextByte());
            return 2;

        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RRC(A));
            clearFlag('Z'); // edge case with 0 flags
            clearFlag('N');
            clearFlag('H');
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
            /*TODO: STOP*/
            uint8_t next = fetchNextByte();
            if(next == 0x00){
                stop = true;
            } // bug with some games, if next byte isnt NOP, treat as NOP
            return 4;
        case (0x1):
            registers[2] = fetchNext2Bytes();
            return 3;

        case (0x2):
            uint8_t A = getUpper(registers[0]);
            uint16_t DE = registers[2];
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
            ldReg8('A', RL(A));
            clearFlag('Z'); // edge case with 0 flags
            clearFlag('N');
            clearFlag('H');
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
            ldReg8('C', fetchNextByte());
            return 2;

        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RR(A));
            clearFlag('Z'); // edge case with 0 flags
            clearFlag('N');
            clearFlag('H');
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
            if (!getFlag('Z'))
            {
                pc += fetchNextByte();
                return 3;
            }
            else
            {
                pc++; //  skip intermediate instruction
                return 2;
            }

        case (0x1):
            registers[3] = fetchNext2Bytes();
            return 3;

        case (0x2):
            uint8_t A = getUpper(registers[0]);
            uint16_t &HL = registers[3];
            ldMem8(HL++, A);
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
            uint8_t lower_nibble = A & 0xF;
            uint8_t upper_nibble = A & 0xF0;
            if (getFlag('N'))
            {
                if (getFlag('C'))
                {
                    A -= 0x60;
                }
                if (getFlag('H'))
                {
                    A -= 0x06;
                }
            }
            else
            {
                if (getFlag('C') || upper_nibble > 0x99)
                {
                    A += 0x60;
                    setFlag('C');
                }
                if (getFlag('H') || lower_nibble > 0x09)
                {
                    A += 0x06;
                }
            }

            if (A == 0)
            {
                setFlag('Z');
            }
            else
            {
                clearFlag('Z');
            }

            clearFlag('H');

            ldReg8('A', A);
            return 1;

        case (0x8):
            if (getFlag('Z'))
            {
                pc += fetchNextByte();
                return 3;
            }
            else
            {
                pc++;
                return 2;
            }

        case (0x9):
            uint16_t HL = registers[3];
            registers[3] = add16(HL, HL);
            return 2;

        case (0xA):
            uint16_t &HL = registers[3];
            ldReg8('A', HL++);
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
            ldReg8('L', fetchNextByte());
            return 2;

        case (0xF):
            uint8_t A = ~getUpper(registers[0]);
            ldReg8('A', A);
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
            if (!getFlag('C'))
            {
                pc += fetchNextByte();
                return 3;
            }
            else
            {
                pc++;
                return 2;
            }

        case (0x1):
            sp = fetchNext2Bytes();
            return 3;

        case (0x2):
            uint8_t A = getUpper(registers[0]);
            uint16_t &HL = registers[3];
            ldMem8(HL--, A);
            return 2;

        case (0x3):
            sp++;
            return 2;

        case (0x4):
            uint8_t data = MMU.readMem(registers[3]) + 1;
            MMU.writeMem(data, registers[3]);
            return 2;

        case (0x5):
            uint8_t data = MMU.readMem(registers[3]) - 1;
            MMU.writeMem(data, registers[3]);
            return 2;

        case (0x6):
            MMU.writeMem(fetchNextByte(), registers[3]);
            return 3;
        case (0x7):
            setFlag('C');
            return 1;

        case (0x8):
            if (getFlag('C'))
            {
                pc += fetchNextByte();
                return 3;
            }
            else
            {
                pc++;
                return 2;
            }

        case (0x9):
            uint16_t HL = registers[3];
            registers[3] = add16(HL, sp);
            return 2;

        case (0xA):
            uint16_t &HL = registers[3];
            ldReg8('A', HL--);
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
            ldReg8('A', fetchNextByte());
            return 2;

        case (0xF):
            if (getFlag('C'))
            {
                clearFlag('C');
            }
            else
            {
                setFlag('C');
            }
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
            ldReg8('B', B);
            return 1;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('B', C);
            return 1;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('B', D);
            return 1;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('B', E);
            return 1;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('B', H);
            return 1;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('B', L);
            return 1;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('B', HL_data);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('B', A);
            return 1;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('C', B);
            return 1;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', C);
            return 1;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('C', D);
            return 1;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('C', E);
            return 1;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('C', H);
            return 1;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('C', L);
            return 1;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('C', HL_data);
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
            ldReg8('D', B);
            return 1;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('D', C);
            return 1;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', D);
            return 1;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('D', E);
            return 1;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('D', H);
            return 1;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('D', L);
            return 1;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('D', HL_data);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('D', A);
            return 1;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('E', B);
            return 1;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('E', C);
            return 1;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('E', D);
            return 1;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', E);
            return 1;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('E', H);
            return 1;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('E', L);
            return 1;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('E', HL_data);
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
            ldReg8('H', B);
            return 1;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('H', C);
            return 1;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('H', D);
            return 1;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('H', E);
            return 1;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', H);
            return 1;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('H', L);
            return 1;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('H', HL_data);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('H', A);
            return 1;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('L', B);
            return 1;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('L', C);
            return 1;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('L', D);
            return 1;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('L', E);
            return 1;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('L', H);
            return 1;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', L);
            return 1;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('L', HL_data);
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
            ldMem8(HL, B);
            return 2;
        case (0x1):
            uint16_t HL = registers[3];
            uint8_t C = getLower(registers[1]);
            ldMem8(HL, C);
            return 2;
        case (0x2):
            uint16_t HL = registers[3];
            uint8_t D = getUpper(registers[2]);
            ldMem8(HL, D);
            return 2;
        case (0x3):
            uint16_t HL = registers[3];
            uint8_t E = getLower(registers[2]);
            ldMem8(HL, E);
            return 2;
        case (0x4):
            uint16_t HL = registers[3];
            uint8_t H = getUpper(registers[3]);
            ldMem8(HL, H);
            return 2;
        case (0x5):
            uint16_t HL = registers[3];
            uint8_t L = getLower(registers[3]);
            ldMem8(HL, L);
            return 2;
        case (0x6):
            /*TODO: HALT*/
            halt = true;
            return 4;
        case (0x7):
            uint16_t HL = registers[3];
            uint8_t A = getUpper(registers[0]);
            ldMem8(HL, A);
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('A', B);
            return 1;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('A', C);
            return 1;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('A', D);
            return 1;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('A', E);
            return 1;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('A', H);
            return 1;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('A', L);
            return 1;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldReg8('A', HL_data);
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
            uint8_t B = getUpper(registers[1]);
            addA(B);
            return 1;

        case (0x1):
            uint8_t C = getLower(registers[1]);
            addA(C);
            return 1;

        case (0x2):
            uint8_t D = getUpper(registers[2]);
            addA(D);
            return 1;

        case (0x3):
            uint8_t E = getLower(registers[2]);
            addA(E);
            return 1;

        case (0x4):
            uint8_t H = getUpper(registers[3]);
            addA(H);
            return 1;

        case (0x5):
            uint8_t L = getLower(registers[3]);
            addA(L);
            return 1;

        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            addA(HL_data);
            return 2;

        case (0x7):
            uint8_t A = getUpper(registers[0]);
            addA(A);
            return 1;

        case (0x8):
            uint8_t B = getUpper(registers[1]);
            adcA(B);
            return 1;

        case (0x9):
            uint8_t C = getLower(registers[1]);
            adcA(C);
            return 1;

        case (0xA):
            uint8_t D = getUpper(registers[2]);
            adcA(D);
            return 1;

        case (0xB):
            uint8_t E = getLower(registers[2]);
            adcA(E);
            return 1;

        case (0xC):
            uint8_t H = getUpper(registers[3]);
            adcA(H);
            return 1;

        case (0xD):
            uint8_t L = getLower(registers[3]);
            adcA(L);
            return 1;

        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            adcA(HL_data);
            return 2;

        case (0xF):
            uint8_t A = getUpper(registers[0]);
            adcA(A);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x9):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            subA(B);
            return 1;

        case (0x1):
            uint8_t C = getLower(registers[1]);
            subA(C);
            return 1;

        case (0x2):
            uint8_t D = getUpper(registers[2]);
            subA(D);
            return 1;

        case (0x3):
            uint8_t E = getLower(registers[2]);
            subA(E);
            return 1;

        case (0x4):
            uint8_t H = getUpper(registers[3]);
            subA(H);
            return 1;

        case (0x5):
            uint8_t L = getLower(registers[3]);
            subA(L);
            return 1;

        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            subA(HL_data);
            return 2;

        case (0x7):
            uint8_t A = getUpper(registers[0]);
            subA(A);
            return 1;

        case (0x8):
            uint8_t B = getUpper(registers[1]);
            sbcA(B);
            return 1;

        case (0x9):
            uint8_t C = getLower(registers[1]);
            sbcA(C);
            return 1;

        case (0xA):
            uint8_t D = getUpper(registers[2]);
            sbcA(D);
            return 1;

        case (0xB):
            uint8_t E = getLower(registers[2]);
            sbcA(E);
            return 1;

        case (0xC):
            uint8_t H = getUpper(registers[3]);
            sbcA(H);
            return 1;

        case (0xD):
            uint8_t L = getLower(registers[3]);
            sbcA(L);
            return 1;

        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            sbcA(HL_data);
            return 2;

        case (0xF):
            uint8_t A = getUpper(registers[0]);
            sbcA(A);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xA):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            andA(B);
            return 1;

        case (0x1):
            uint8_t C = getLower(registers[1]);
            andA(C);
            return 1;

        case (0x2):
            uint8_t D = getUpper(registers[2]);
            andA(D);
            return 1;

        case (0x3):
            uint8_t E = getLower(registers[2]);
            andA(E);
            return 1;

        case (0x4):
            uint8_t H = getUpper(registers[3]);
            andA(H);
            return 1;

        case (0x5):
            uint8_t L = getLower(registers[3]);
            andA(L);
            return 1;

        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            andA(HL_data);
            return 2;

        case (0x7):
            uint8_t A = getUpper(registers[0]);
            andA(A);
            return 1;

        case (0x8):
            uint8_t B = getUpper(registers[1]);
            xorA(B);
            return 1;

        case (0x9):
            uint8_t C = getLower(registers[1]);
            xorA(C);
            return 1;

        case (0xA):
            uint8_t D = getUpper(registers[2]);
            xorA(D);
            return 1;

        case (0xB):
            uint8_t E = getLower(registers[2]);
            xorA(E);
            return 1;

        case (0xC):
            uint8_t H = getUpper(registers[3]);
            xorA(H);
            return 1;

        case (0xD):
            uint8_t L = getLower(registers[3]);
            xorA(L);
            return 1;

        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            xorA(HL_data);
            return 2;

        case (0xF):
            uint8_t A = getUpper(registers[0]);
            xorA(A);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xB):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            orA(B);
            return 1;

        case (0x1):
            uint8_t C = getLower(registers[1]);
            orA(C);
            return 1;

        case (0x2):
            uint8_t D = getUpper(registers[2]);
            orA(D);
            return 1;

        case (0x3):
            uint8_t E = getLower(registers[2]);
            orA(E);
            return 1;

        case (0x4):
            uint8_t H = getUpper(registers[3]);
            orA(H);
            return 1;

        case (0x5):
            uint8_t L = getLower(registers[3]);
            orA(L);
            return 1;

        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            orA(HL_data);
            return 2;

        case (0x7):
            uint8_t A = getUpper(registers[0]);
            orA(A);
            return 1;

        case (0x8):
            uint8_t B = getUpper(registers[1]);
            cpA(B);
            return 1;

        case (0x9):
            uint8_t C = getLower(registers[1]);
            cpA(C);
            return 1;

        case (0xA):
            uint8_t D = getUpper(registers[2]);
            cpA(D);
            return 1;

        case (0xB):
            uint8_t E = getLower(registers[2]);
            cpA(E);
            return 1;

        case (0xC):
            uint8_t H = getUpper(registers[3]);
            cpA(H);
            return 1;

        case (0xD):
            uint8_t L = getLower(registers[3]);
            cpA(L);
            return 1;

        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            cpA(HL_data);
            return 2;

        case (0xF):
            uint8_t A = getUpper(registers[0]);
            cpA(A);
            return 1;

        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xC):
    {
        switch (secondNibble)
        {
        case (0x0):
            if (!getFlag('Z'))
            {
                pc = (popStack()) | (popStack() << 8);
                return 5;
            }
            else
            {
                return 2;
            }
        case (0x1):
            uint8_t C = popStack();
            uint8_t B = popStack();

            registers[1] = (B << 8) | C;
            return 3;
        case (0x2):
            if (!getFlag('Z'))
            {
                pc = fetchNext2Bytes();
                return 4;
            }
            else
            {
                pc += 2;
                return 3;
            }
        case (0x3):
            pc = fetchNext2Bytes();
            return 4;
        case (0x4):
            if (!getFlag('Z'))
            {
                storePC();
                pc = fetchNext2Bytes();
                return 6;
            }
            else
            {
                pc += 2;
                return 3;
            }
        case (0x5):
            uint8_t B = getUpper(registers[1]);
            uint8_t C = getLower(registers[1]);
            pushStack(B);
            pushStack(C);
            return 4;
        case (0x6):
            addA(fetchNextByte());
            return 2;
        case (0x7):
            storePC();
            pc = 0x00;
            return 4;
        case (0x8):
            if (getFlag('Z'))
            {
                pc = popStack() | (popStack() << 8);
                return 5;
            }
            else
            {
                return 2;
            }
        case (0x9):
            pc = popStack() | (popStack() << 8);
            return 4;
        case (0xA):
            if (getFlag('Z'))
            {
                pc = fetchNext2Bytes();
                return 4;
            }
            else
            {
                pc += 2;
                return 3;
            }
        case (0xB):
            return 0; /// nan

        case (0xC):
            if (getFlag('Z'))
            {
                storePC();
                pc = fetchNext2Bytes();
                return 6;
            }
            else
            {
                pc += 2;
                return 3;
            }
        case (0xD):
            storePC();
            pc = fetchNext2Bytes();
            return 6;
        case (0xE):
            adcA(fetchNextByte());
            return 2;
        case (0xF):
            storePC();
            pc = 0x08;
            return 4;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xD):
    {
        switch (secondNibble)
        {
        case (0x0):
            if (!getFlag('C'))
            {
                pc = (popStack()) | (popStack() << 8);
                return 5;
            }
            else
            {
                return 2;
            }

        case (0x1):
            uint8_t E = popStack();
            uint8_t D = popStack();
            registers[2] = (D >> 8) | E;

        case (0x2):
            if (!getFlag('C'))
            {
                pc = fetchNext2Bytes();
                return 4;
            }
            else
            {
                pc += 2;
                return 3;
            }
        case (0x3):
            return 0; // nan
        case (0x4):
            if (!getFlag('C'))
            {
                storePC();
                pc = fetchNext2Bytes();
                return 6;
            }
            else
            {
                pc += 2;
                return 3;
            }
        case (0x5):
            uint8_t D = getUpper(registers[2]);
            uint8_t E = getLower(registers[2]);
            pushStack(D);
            pushStack(E);
            return 4;
        case (0x6):
            subA(fetchNextByte());
            return 2;
        case (0x7):
            storePC();
            pc = 0x10;
            return 4;
        case (0x8):
            if (getFlag('C'))
            {
                pc = popStack() | (popStack() << 8);
                return 5;
            }
            else
            {
                return 2;
            }
        case (0x9):
            /*TODO: reset Interrupt Flags*/
            pc = popStack() | (popStack() << 8);
            return 4;
        case (0xA):
            if (getFlag('C'))
            {
                pc = fetchNext2Bytes();
                return 4;
            }
            else
            {
                pc += 2;
                return 3;
            }
        case (0xB):
            return 0; // nan
        case (0xC):
            if (getFlag('C'))
            {
                storePC();
                pc = fetchNext2Bytes();
                return 6;
            }
            else
            {
                pc += 2;
                return 3;
            }
        case (0xD):
            return 0; // nan
        case (0xE):
            sbcA(fetchNextByte());
            return 2;
        case (0xF):
            storePC();
            pc = 0x18;
            return 4;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xE):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t A = getUpper(registers[0]);
            uint16_t address = (0xFF00) | fetchNextByte();
            ldMem8(address, A);
            return 3;
        case (0x1):
            uint8_t L = popStack();
            uint8_t H = popStack();

            registers[3] = (H << 8) | L;
            return 3;
        case (0x2):
            uint8_t A = getUpper(registers[0]);
            uint8_t C = getLower(registers[1]);
            uint16_t address = (0xFF00 | C);
            ldMem8(address, A);
            return 2;
        case (0x3):
            return 0;
        case (0x4):
            return 0;
        case (0x5):
            uint8_t H = getUpper(registers[3]);
            uint8_t L = getLower(registers[3]);

            pushStack(H);
            pushStack(L);
            return 4;
        case (0x6):
            andA(fetchNextByte());
            return 2;
        case (0x7):
            storePC();
            pc = 0x20;
            return 4;
        case (0x8):
            int8_t signedNext = static_cast<int8_t>(fetchNextByte()); // cast to signed byte
            sp += signedNext;
            return 4;
        case (0x9):
            pc = registers[3];
            return 1;
        case (0xA):
            uint8_t A = getUpper(registers[0]);
            ldMem8(fetchNext2Bytes(), A);
            return 4;
        case (0xB):
            return 0;
        case (0xC):
            return 0;
        case (0xD):
            return 0;
        case (0xE):
            xorA(fetchNextByte());
            return 2;
        case (0xF):
            storePC();
            pc = 0x28;
            return 4;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xF):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint16_t data = MMU.readMem(fetchNextByte() | (0xFF00));
            ldReg8('A', data);
            return 3;
        case (0x1):
            uint8_t F = popStack();
            uint8_t A = popStack();
            return 3;
        case (0x2):
            uint8_t C = getLower(registers[1]);
            uint16_t address = (0xFF00 | C);
            uint8_t data = MMU.readMem(address);
            ldReg8('A', data);
            return 2;
        case (0x3):
            IME = false;
            return 1;
        case (0x4):
            return 0;
        case (0x5):
            uint8_t A = getUpper(registers[0]);
            uint8_t F = getLower(registers[0]);
            pushStack(A);
            pushStack(F);
            return 4;
        case (0x6):
            orA(fetchNextByte());
            return 2;
        case (0x7):
            storePC();
            pc = 0x30;
            return 4;
        case (0x8):
            int8_t signedNext = static_cast<int8_t>(fetchNextByte());
            registers[3] = sp + signedNext;
            return 3;
        case (0x9):
            sp = registers[3];
            return 2;
        case (0xA):
            ldReg8('A', MMU.readMem(fetchNext2Bytes()));
            return 4;
        case (0xB):
            EI_FLAG = true;
            return 1;
        case (0xC):
            return 0;
        case (0xD):
            return 0;
        case (0xE):
            cpA(fetchNextByte());
            return 2;
        case (0xF):
            storePC();
            pc = 0x38;
            return 4;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    default:
        throw std::runtime_error("Invalid Opcode!");
    }
}

// prefixed helper functions
uint8_t cpu::RLC(uint8_t byte)
{
    bool leftMost = byte >> 7;
    uint8_t temp = (byte << 1) | leftMost;
    if (leftMost)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    };
    if (temp == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');
    return temp;
} // rotate left, store bit in carry
uint8_t cpu::RL(uint8_t byte)
{
    bool leftMost = byte >> 7;
    uint8_t temp = (byte << 1) | getFlag('C');
    // must do before setting flags to get original Flag value
    if (leftMost)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }
    if (temp == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');

    return temp;
} // rotate left, through carry
uint8_t cpu::RRC(uint8_t byte)
{
    bool rightMost = byte & 0x1;
    uint8_t temp = (byte >> 1) | (rightMost << 7);
    if (rightMost)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }
    if (temp == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');

    return temp;
} // rotate right, store bit in carry
uint8_t cpu::RR(uint8_t byte)
{
    bool rightMost = byte & 0x1;
    uint8_t temp = (byte >> 1) | (getFlag('C') << 7);
    if (rightMost)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }
    if (temp == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');

    return temp;
} // rotate right, through carry
uint8_t cpu::SLA(uint8_t byte)
{
    bool leftMost = byte >> 7;
    uint8_t temp = (byte << 1) | 0x0;
    if (leftMost)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }
    if (temp == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');

    return temp;
} // shift left, through carry flag, bit 0 = 0
uint8_t cpu::SRA(uint8_t byte)
{
    bool rightMost = byte & 0x1;
    bool b7 = byte & 0x80; // mask bit 7
    uint8_t temp = (byte >> 1) | (b7);
    if (rightMost)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }
    if (temp == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');

    return temp;
} // shift right, through carry flag, bit 7 unchanged
uint8_t cpu::SWAP(uint8_t byte)
{
    uint8_t upperNibble = byte & 0xF0;
    uint8_t lowerNibble = byte & 0x0F;
    uint8_t temp = (lowerNibble << 4) | (upperNibble >> 4);
    if (temp == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('C');
    clearFlag('N');
    clearFlag('H');

    return temp;
} // swap nibbles
uint8_t cpu::SRL(uint8_t byte)
{
    bool rightMost = byte & 0x1;
    uint8_t temp = byte >> 1;
    if (rightMost)
    {
        setFlag('C');
    }
    else
    {
        clearFlag('C');
    }
    if (temp == 0)
    {
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    clearFlag('H');

    return temp;
} // shift right, through carry flag, bit 7 = 0
void cpu::BIT_(uint8_t byte, int n)
{
    if (!(byte & (0x1 << n)))
    { // mask out opposite of nth bit
        setFlag('Z');
    }
    else
    {
        clearFlag('Z');
    }
    clearFlag('N');
    setFlag('H');
} // copy ! bit _ into Z flag
uint8_t cpu::RES_(uint8_t byte, int n)
{
    uint8_t mask = ~(0x1 << n); // every bit on execept bit n
    return byte & mask;
} // reset bit n
uint8_t cpu::SET_(uint8_t byte, int n)
{
    uint8_t mask = (0x1 << n);
    return byte | mask;
} //  set bit n

int cpu::executePrefixed()
{
    fetchOpcode(); // get last 8 bits of instruction
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
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RLC(B));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RLC(C));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RLC(D));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RLC(E));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RLC(H));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RLC(L));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RLC(HL_data));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RLC(A));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RRC(B));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RRC(C));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RRC(D));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RRC(E));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RRC(H));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RRC(L));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RRC(HL_data));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RRC(A));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x1):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RL(B));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RL(C));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RL(D));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RL(E));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RL(H));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RL(L));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RL(HL_data));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RL(A));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RR(B));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RR(C));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RR(D));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RR(E));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RR(H));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RR(L));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RR(HL_data));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RR(A));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x2):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SLA(B));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SLA(C));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SLA(D));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SLA(E));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SLA(H));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SLA(L));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SLA(HL_data));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SLA(A));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SRA(B));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SRA(C));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SRA(D));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SRA(E));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SRA(H));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SRA(L));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SRA(HL_data));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SRA(A));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x3):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SWAP(B));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SWAP(C));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SWAP(D));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SWAP(E));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SWAP(H));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SWAP(L));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SWAP(HL_data));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SWAP(A));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SRL(B));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SRL(C));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SRL(D));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SRL(E));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SRL(H));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SRL(L));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SRL(HL_data));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SRL(A));
            return 2;
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
            BIT_(B, 0);
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            BIT_(C, 0);
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            BIT_(D, 0);
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            BIT_(E, 0);
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            BIT_(H, 0);
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            BIT_(L, 0);
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            BIT_(HL_data, 0);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            BIT_(A, 0);
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            BIT_(B, 1);
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            BIT_(C, 1);
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            BIT_(D, 1);
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            BIT_(E, 1);
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            BIT_(H, 1);
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            BIT_(L, 1);
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            BIT_(HL_data, 1);
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            BIT_(A, 1);
            return 2;
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
            BIT_(B, 2);
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            BIT_(C, 2);
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            BIT_(D, 2);
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            BIT_(E, 2);
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            BIT_(H, 2);
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            BIT_(L, 2);
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            BIT_(HL_data, 2);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            BIT_(A, 2);
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            BIT_(B, 3);
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            BIT_(C, 3);
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            BIT_(D, 3);
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            BIT_(E, 3);
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            BIT_(H, 3);
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            BIT_(L, 3);
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            BIT_(HL_data, 3);
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            BIT_(A, 3);
            return 2;
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
            BIT_(B, 4);
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            BIT_(C, 4);
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            BIT_(D, 4);
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            BIT_(E, 4);
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            BIT_(H, 4);
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            BIT_(L, 4);
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            BIT_(HL_data, 4);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            BIT_(A, 4);
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            BIT_(B, 5);
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            BIT_(C, 5);
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            BIT_(D, 5);
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            BIT_(E, 5);
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            BIT_(H, 5);
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            BIT_(L, 5);
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            BIT_(HL_data, 5);
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            BIT_(A, 5);
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x7):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            BIT_(B, 6);
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            BIT_(C, 6);
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            BIT_(D, 6);
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            BIT_(E, 6);
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            BIT_(H, 6);
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            BIT_(L, 6);
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            BIT_(HL_data, 6);
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            BIT_(A, 6);
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            BIT_(B, 7);
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            BIT_(C, 7);
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            BIT_(D, 7);
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            BIT_(E, 7);
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            BIT_(H, 7);
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            BIT_(L, 7);
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            BIT_(HL_data, 7);
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            BIT_(A, 7);
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x8):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RES_(B, 0));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RES_(C, 0));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RES_(D, 0));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RES_(E, 0));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RES_(H, 0));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RES_(L, 0));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RES_(HL_data, 0));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RES_(A, 0));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RES_(B, 1));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RES_(C, 1));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RES_(D, 1));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RES_(E, 1));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RES_(H, 1));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RES_(L, 1));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RES_(HL_data, 1));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RES_(A, 1));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0x9):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RES_(B, 2));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RES_(C, 2));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RES_(D, 2));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RES_(E, 2));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RES_(H, 2));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RES_(L, 2));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RES_(HL_data, 2));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RES_(A, 2));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RES_(B, 3));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RES_(C, 3));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RES_(D, 3));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RES_(E, 3));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RES_(H, 3));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RES_(L, 3));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RES_(HL_data, 3));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RES_(A, 3));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xA):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RES_(B, 4));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RES_(C, 4));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RES_(D, 4));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RES_(E, 4));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RES_(H, 4));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RES_(L, 4));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RES_(HL_data, 4));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RES_(A, 4));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RES_(B, 5));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RES_(C, 5));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RES_(D, 5));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RES_(E, 5));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RES_(H, 5));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RES_(L, 5));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RES_(HL_data, 5));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RES_(A, 5));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xB):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RES_(B, 6));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RES_(C, 6));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RES_(D, 6));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RES_(E, 6));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RES_(H, 6));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RES_(L, 6));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RES_(HL_data, 6));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RES_(A, 6));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', RES_(B, 7));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', RES_(C, 7));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', RES_(D, 7));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', RES_(E, 7));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', RES_(H, 7));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', RES_(L, 7));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], RES_(HL_data, 7));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', RES_(A, 7));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xC):
    {
        switch (secondNibble)
        {

        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SET_(B, 0));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SET_(C, 0));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SET_(D, 0));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SET_(E, 0));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SET_(H, 0));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SET_(L, 0));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SET_(HL_data, 0));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SET_(A, 0));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SET_(B, 1));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SET_(C, 1));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SET_(D, 1));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SET_(E, 1));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SET_(H, 1));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SET_(L, 1));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SET_(HL_data, 1));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SET_(A, 1));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xD):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SET_(B, 2));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SET_(C, 2));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SET_(D, 2));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SET_(E, 2));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SET_(H, 2));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SET_(L, 2));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SET_(HL_data, 2));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SET_(A, 2));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SET_(B, 3));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SET_(C, 3));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SET_(D, 3));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SET_(E, 3));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SET_(H, 3));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SET_(L, 3));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SET_(HL_data, 3));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SET_(A, 3));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xE):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SET_(B, 4));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SET_(C, 4));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SET_(D, 4));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SET_(E, 4));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SET_(H, 4));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SET_(L, 4));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SET_(HL_data, 4));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SET_(A, 4));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SET_(B, 5));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SET_(C, 5));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SET_(D, 5));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SET_(E, 5));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SET_(H, 5));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SET_(L, 5));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SET_(HL_data, 5));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SET_(A, 5));
            return 2;
        default:
            throw std::runtime_error("Invalid Opcode!");
        }
    }

    case (0xF):
    {
        switch (secondNibble)
        {
        case (0x0):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SET_(B, 6));
            return 2;
        case (0x1):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SET_(C, 6));
            return 2;
        case (0x2):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SET_(D, 6));
            return 2;
        case (0x3):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SET_(E, 6));
            return 2;
        case (0x4):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SET_(H, 6));
            return 2;
        case (0x5):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SET_(L, 6));
            return 2;
        case (0x6):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SET_(HL_data, 6));
            return 2;
        case (0x7):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SET_(A, 6));
            return 2;
        case (0x8):
            uint8_t B = getUpper(registers[1]);
            ldReg8('B', SET_(B, 7));
            return 2;
        case (0x9):
            uint8_t C = getLower(registers[1]);
            ldReg8('C', SET_(C, 7));
            return 2;
        case (0xA):
            uint8_t D = getUpper(registers[2]);
            ldReg8('D', SET_(D, 7));
            return 2;
        case (0xB):
            uint8_t E = getLower(registers[2]);
            ldReg8('E', SET_(E, 7));
            return 2;
        case (0xC):
            uint8_t H = getUpper(registers[3]);
            ldReg8('H', SET_(H, 7));
            return 2;
        case (0xD):
            uint8_t L = getLower(registers[3]);
            ldReg8('L', SET_(L, 7));
            return 2;
        case (0xE):
            uint8_t HL_data = MMU.readMem(registers[3]);
            ldMem8(registers[3], SET_(HL_data, 7));
            return 2;
        case (0xF):
            uint8_t A = getUpper(registers[0]);
            ldReg8('A', SET_(A, 7));
            return 2;
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

int cpu::handleInterrupts()
{
    uint8_t IE = MMU.readMem(0xFFFF);
    uint8_t IF = MMU.readMem(0xFF0F);
    uint8_t pending = IE & IF;
    // 0001 1111
    if(IME){  
        if (pending & 0x1) // any positive number will evaluate to true, mask out highest priority first
        {   
            IME = false;
            MMU.writeMem(IF & ~0x1,0xFF0F);
            storePC();
            pc = 0x40;
            return 5;
        }
        else if (pending & 0x2)
        {
            IME = false;
            MMU.writeMem(IF & ~0x2,0xFF0F);
            storePC();
            pc = 0x48;
            return 5;
        }
        else if (pending & 0x4)
        {
            IME = false;
        MMU.writeMem(IF & ~0x4,0xFF0F);
            storePC();
            pc = 0x50;
            return 5;
        }
        else if(pending & 0x8)
        {
            IME = false;
            MMU.writeMem(IF & ~0x8,0xFF0F);
            storePC();
            pc = 0x58;
            return 5;
        }
        else if(pending & 0x10)
        {
            IME = false;
            MMU.writeMem(IF & ~0x10,0xFF0F);
            storePC();
            pc = 0x60;
            return 5;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}
// Constructor
cpu::cpu(mmu &MMUref, ppu &PPUref, timer &TIMERref) : MMU(MMUref), PPU(PPUref), TIMER(TIMERref)
{
    memset(registers, 0, sizeof(registers));
    pc = 0x0100; // Right above boot rom
    sp = 0xFFFE; // only lives in high ram for boot, then moves to work ram, downwardly
    opcode = 0;
    IME = false;
    stop = false;
    halt = false;
}

// Game loop
int cpu::step()
{   
    // handle stop
    // handle halt
    int interruptCycles = handleInterrupts();
    cycles += interruptCycles;
    if(interruptCycles == 0){
        cycles += executeOpcode();
    }
    if(EI_FLAG){
        IME = true;
        EI_FLAG = false;
    }

    return cycles;

    // return cycles to help sync with ppu and timer
}
