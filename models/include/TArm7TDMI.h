/** 
 * This file is part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
 *
 * Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
 * 
 * This file is adapted from HF-RISC SoC project, which can be found at johanns' 
 * reposiitory at GitHub: https://github.com/sjohann81/hf-risc
 *-------------------------------------------------------------------------------
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. 
 *---------------------------------------------------------------------------- */
#ifndef _TARM7TDMI_H
#define _TARM7TDMI_H

#include <cstdlib>
#include <sstream>

#include "TimedModel.h"

//supported data types
typedef enum{
    BYTE = 8,
    HALF = 16,
    WORD = 32
} ArmDataTypes;

//processor modes 
typedef enum{
    USER = 0x10, /* 1 0000b */
    FIQ  = 0x11, /* 1 0001b */
    IRQ  = 0x12, /* 1 0010b */
    SVC  = 0x13, /* 1 0011b */
    ABT  = 0x17, /* 1 0111b */ 
    UND  = 0x1B /* 1 1011b */
} ArmProcModes;

//Banked registers (general purpose regs)
#define R0  _registers[0]
#define R2  _registers[2]
#define R1  _registers[1]
#define R3  _registers[3]
#define R4  _registers[4]
#define R5  _registers[5]
#define R6  _registers[6]
#define R7  _registers[7]
#define R8  _registers[8]
#define R9  _registers[9]
#define R10 _registers[10]
#define R11 _registers[11]
#define R12 _registers[12]

//SP (stack pointer)
#define R13 _registers[13]
#define SP  _registers[13]

//LR (link register) 
#define R14 _registers[14]
#define LR  _registers[14]

//PC 
#define R15 _registers[15]
#define PC  _registers[15]

#define CSPR _registers[16]

#define R13SVC _registers[17]
#define R14SVC _registers[18]
#define R13ABT _registers[19]
#define R14ABT _registers[20]
#define R13UND _registers[21]
#define R14UND _registers[22]
#define R13IRQ _registers[23]
#define R14IRQ _registers[24]

#define R8FIQ  _registers[25]
#define R9FIQ  _registers[26]
#define R10FIQ _registers[27]
#define R11FIQ _registers[28]
#define R12FIQ _registers[29]
#define R13FIQ _registers[30]
#define R14FIQ _registers[31]

#define SPSRSVC _registers[32]
#define SPSRABT _registers[33]
#define SPSRUND _registers[34]
#define SPSRIRQ _registers[35]
#define SPSRFIQ _registers[36]

//PSR bits
#define PSR_N   /* negative */
#define PSR_Z   /* zero */
#define PSR_C   /* carry */
#define PSR_V   /* overflow */
#define PSR_Q   /* saturated integer arithmetic -- reserved if prior to ARMv5 */
#define PSR_RESERVED
#define PSR_GE  /* greater than or equals -- ARMv6 and above */
#define PSR_E
#define PSR_A   /* imprecise abort mask -- ARMv6 specific */
#define PSR_I
#define PSR_F
#define PSR_T   /* thumb mode state -- 4T and above */
#define PSR_M   /* processor mode (from modes) */


/**
 * This class implements a model for an ARM7-TDMI processor, which 
 * implements the ARM-V4T instruction set (ARM v.4 plus THUMB v.1
 * instructions. ARM ISA covers 32-bit width instructions, 
 * and THUMB ISA covers 16-bit and 8-bit width instructions.
 */
class TArm7TDMI : TimedModel{

    private:
        int32_t _registers[37]; /* all registers' array */
		
        int32_t fetch();
        int32_t decode();
        int32_t execute();

    public:
		void Reset();
		TArm7TDMI(string name);
		~TArm7TDMI();
        long long unsigned int Run();
};
#endif /* _TARM7TDMI_H */

