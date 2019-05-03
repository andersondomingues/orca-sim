/** 
 * This file is part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
 *
 * Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
 * 
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

enum ArmConds{
              // FLAGS              MEANING
	EQ = 0x0, // Z set              equals
	NE = 0x1, // Z clear            not equals
	CS = 0x2, // C set              unsigned higher or same
	CC = 0x3, // C clear            unsigned lower
	MI = 0x4, // N set              negative
	PL = 0x5, // N clear            positive or zero
	VS = 0x6, // V set              overflow
	VC = 0x7, // V clear            not overflow
	HI = 0x8, // C set and Z clear  unsigned higher
	LS = 0x9, // C clearor Z clear  unsigned lower or same
	GE = 0xA, // N = V              greater of equals
	LT = 0xB, // N not equals V     less than
	GT = 0xC, // Z clear and (N=V)  greater than
	LE = 0xD, // Z ser or (N!=V)    lees than or equal
	AL = 0xE  // (ignored)          always
}

//processor modes 
typedef enum{
    USER = 0x10, /* 1 0000b */
    FIQ  = 0x11, /* 1 0001b */
    IRQ  = 0x12, /* 1 0010b */
    SVC  = 0x13, /* 1 0011b */
    ABT  = 0x17, /* 1 0111b */ 
    UND  = 0x1B /* 1 1011b */
} ArmProcModes;

//--------------------------------------------
// Instructions
//--------------------------------------------

//moves
#define MOV

//arithmetic
#define ADD
#define ADC /* add with carry */
#define SUB
#define SBC /* sub with carry */
#define NEG /* negate */
#define MUL /* multiply */
#define CMP /* compare */
#define CMN /* compare negative */

//logical
#define AND 
#define EOR
#define OR
#define BIC /* bit clear */
#define MVN /* move not */
#define TST /* test bits */

//shifts/rotate
#define LSL /* logical shift left */
#define LSR /* logical shift right */
#define ASR /* arithmetic shift right */
#define ROR /* rotate right */

//branches
#define BEQ
#define BNE
#define BCS
#define BCC
#define BMI
#define BPL
#define BVS
#define BVC
#define BHI
#define BLS
#define BGE
#define BLT
#define BGT
#define BLE
#define B
#define BL
#define BX /* optional state change */

//loades
#define LDR
#define LDRH
#define LDRB
#define LDRSH
#define LDRSB
#define LDMIA

//store
#define STR
#define STRH
#define STRB
#define STMIA

//push/pop
#define PUSH
#define POP

//software interrupt
#define SWI


//--------------------------------------------
// Banked registers (general purpose regs)
//--------------------------------------------
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
#define R13 _registers[13] /* SP (stack pointer) */
#define SP  _registers[13]
#define R14 _registers[14] /* LR (link register) */
#define LR  _registers[14]
#define R15 _registers[15] /* PC (program counter) */
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
#define SPSRSVC _registers[32] /* ?? */
#define SPSRABT _registers[33]
#define SPSRUND _registers[34]
#define SPSRIRQ _registers[35]
#define SPSRFIQ _registers[36]

//PSR bits - Current Program Status Register
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
 * and THUMB ISA covers 16-bit and 8-bit width instructions. TDMI 
 * stands for "16 bit [T]humb + JTAG [D]ebug + fast [M]ultiplier
 * + enhanced [I]CE".
 */
class TArm7TDMI : TimedModel{

    private:
        int32_t _registers[37]; /* all registers' array */
		
        int32_t _fetch();
        int32_t _decode();
        int32_t _execute();

    public:
		void Reset();
		TArm7TDMI(string name);
		~TArm7TDMI();
        long long unsigned int Run();
};
#endif /* _TARM7TDMI_H */

