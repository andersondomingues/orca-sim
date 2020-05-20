/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software-tools
 * http://https://github.com/andersondomingues/orca-mpsoc
 *
 * Copyright (C) 2018-2020 Anderson Domingues, <ti.andersondomingues@gmail.com>
 *
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
******************************************************************************/
#ifndef _TORCAAT_H
#define _TORCAAT_H

#include <cstdlib>
#include <sstream>

#include <TProcessorBase.h>

//supported data types
enum OAT_DataTypes{
    BYTE = 8,
    HALF = 16,
    WORD = 32
};

enum OAT_Conds{
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

//--------------------------------------------
// Banked registers (general purpose regs)
//--------------------------------------------
#define REGS (GetState()->regs)

#define R0  (REGS[0])
#define R1  (REGS[1])
#define R2  (REGS[2])
#define R3  (REGS[3])
#define R4  (REGS[4])
#define R5  (REGS[5])
#define R6  (REGS[6])
#define R7  (REGS[7])
#define R8  (REGS[8])
#define R9  (REGS[9])
#define R10 (REGS[10])
#define R11 (REGS[11])
#define R12 (REGS[12])

#define R13 (REGS[13])
#define SP  (REGS[13])

#define R14 (REGS[14])
#define LR  (REGS[14])

#define R15 (REGS[15])
#define PC  (REGS[15])

#define CSPR   (REGS[16])
#define R13SVC (REGS[17])
#define R14SVC (REGS[18])
#define R13ABT (REGS[19])
#define R14ABT (REGS[20])
#define R13UND (REGS[21])
#define R14UND (REGS[22])
#define R13IRQ (REGS[23])
#define R14IRQ (REGS[24])
#define R8FIQ  (REGS[25])
#define R9FIQ  (REGS[26])
#define R10FIQ (REGS[27])
#define R11FIQ (REGS[28])
#define R12FIQ (REGS[29])
#define R13FIQ (REGS[30])
#define R14FIQ (REGS[31])
#define SPSRSVC (REGS[32])
#define SPSRABT (REGS[33])
#define SPSRUND (REGS[34])
#define SPSRIRQ (REGS[35])
#define SPSRFIQ (REGS[36])

#undef REGS

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



enum OAT_OP{
	// branches
	B,				// branch to target address
	CBNZ, CBZ,		// branch zero, branch not zero
	BL, BLX,		// call subroutine, call subroutine and change ISA 
	BX, 			// branch and change ISA 
	BXJ,			// branch and change to J state 
	TBB, TBH,		// table branches (byte and half-word) 
	// data-processing 
	ADD, ADC,		// add and add with carry 
	ADR,			// form pc-relative address 
	AND,			// bitwise AND 
	BIC,			// bitwise clear 
	CMP, CMN,		// compare, compare negative 
	EOR,			// bitwise XOR 
	MOV, MVN,		// move and move not 
	ORR, ORN,		// btwise, OR, and OR NOT 
	SUB, RSB, RSC,	// subtraction, reverse subtraction and subtraction with carry 
	TST, TEQ,		// test and test equivalence 
	// shifts 
	ASR, 			// arithmetic shift right 
	LSL, LSR,		// logical shift left and right
	ROR, RRX,		// rotate right, rotate right and extend
	// multiplication (general)
	MUL, MLA, MLS, 	// multiply, multiply and accumulate, multiply and subtract
	// multiplication (signed)
	SMLABB, SMLABT,	// multiply accumulative (half)
	SMLATB, SMLATT, 
	SMLAD, SMLAL,	// multiply accumulative (dual and long)
	SMLALBB, SMLALBT,
	SMLALTB, SMLALTT, // multiply accumulative (long half)
	SMLALD,			// multiply accumulative (long dual)
	SMLAWB, SMLAWT, // multiply accumulative (word by half)
	SMLSD, SMSLSLD, // multiply subtract and multiply subtract dual
	SMMUL,			// most significant word multiply
	SMMLA, SMMLS,	// most significant word multiply accumulate and subtract
	SMUAD,			// dual multiply add
	SMUSD,			// dual multiply subtract
	SMULBB, SMULBT,
	SMULTB, SMULTT, // multiply (half)
	SMULL, 			// multiply (long)
	SMULWT, SMULBW, // multiply (word by half)
	// multiplication (unsigned)
	UMULL,			// multiply long
	UMLAL,			// multiply accumulative long
	UMAAL,			// mlutiply accumulative acumulative long 
	//saturation
	SSAT, SSAT16,	// saturate signed 32-bit value, saturate two signed 16-bit values
	USAT, USAT16,   // same as saturate but for unsigned 
	QADD, QSUB,		// add and saturate 32-bit, subtract and saturate 32-bit
	QDADD, QDSUB,	// doubles first, adds to or subtracts from second
	//packing (signed)
	PKH,			// combine half words
	SXTAB, SXTAB16,	// extend 8 to (16 or 32) and add
	SXTAH, 			// extend 16 to 32 and add
	SXTB, SXTB16,	// extend 8 to 32, dual extend 8 to 16
	SXTH,			// extend 16 to 32
	//packing (unsigned)
	UXTAB, UXTAB16,	// extend 8 to 32 and add, dual extend 8 to 16 and add
	UXTAH,			// extend 16 to 32 and add
	UXTB, UXTB16,	// extend 8 to 32, dual extend 8 to 16
	UXTH,			// extend 16 to 32
	//parallel addition and subtraction
	ADD16, 						// add two half
	SADD16, QADD16,	SHADD16,	// signed add two half, add two half saturating, add two half halving 
	UADD16, UQADD16, UHADD16,	// same as above but unsigned
	ASX,				//add ans subtract with exchange
	SASX, QASX, SHASX,  //same for signed, saturating and halving
	USAX, UQSAX, UHASX, //same for unsigned, unsigned saturating, and unsigned halving
	SAX,				//subtract and add with exachange
	SSAX, QSAX, SHSAX,	//same for signed, saturating and halving
	USAX, UQSAX, UHSAX, //same for unsigned, unsigned saturating, and unsigned halving
	ADD8,				//add 4 bytes
	SADD8, QADD8, SHADD8, //same for signed, saturating and halving
	UADD8, UQADD8, UHADD8,//same for unsigned, unsigned saturating, and unsigned halving
	SUB8,				//subtract 4 bytes
	SSUB8, QSUB8, SHSUB8, //same for signed, saturating and halving
	USUB8, UQSUB8, UHSUB8,//same for unsigned, unsigned saturating, and unsigend halving
	//division
	UDIV, SDIV, //unsigned and signed division
	//misc data manipulation
	BFC, BFI,	//bit field clear, bit field insert
	CLZ,		//count leading zeroes
	MOVT,		//move top
	RBTI,		//reverse bits
	REV, REV16, REVSH, //byte-reverse word, packet halfword, and signed halfword
	SBFX, UBFX,	//signed and unsigned bit field extract
	SEL,		//select bytes (uses GE flags)
	USAD8, USADA8, //unsigned sum of absolute diffences and its accumulative version
	//status register access
	MRS, MSR, //move from APSR to register
	CPS,
	//load and store 
	LDR, LDRT, LDREX, 		//load 32-bit, unprivileged, and exclusive
	STR, STRT, STREX, 		//store 32-bit, unprivileged, and exclusive
	STRH, STRHT, STREXH, 	//store 16-bit halfword, unprivileged, and exclusive
	LDRH, LDRHT, LDREXH, 	//load 16-bit unsigned halfword, unprivileged, and exclusive,
	LDRSH, LDRST, 			//load 16-bit signed halfword, unprivileged
	STRB, STRBT, STREXB,	//store 8-bit, unprivileged, exclusive
	LDRB, LDRBT, LDREXB, 	//load 8-bit unsigned, unprivileged, exclusive
	LDRSB, LDRSBT, 			//load 8-byt signed, unprivileged
	LDRD, STRD,				//load two 32-bit word, store two 32-bit word
	LDREXD, STREXD,			//load 64-bit doubleword, store 64-bit doubleword
	//load and store multiple
	LDM, 				//load multiple
	LDMIA, LDMFD,		//same for increment after and full descending
	LDMDA, LDMFA,		//same for decrement after and full ascending
	LDMDB, LDMEA,		//same for decrement before and empty ascending
	LDMIB, LDMED,		//same for increment before and empty descending
	STM, 				//store multiple
	STMIA, STMEA,		//same for increment after and empty ascending
	STMDA, STMED,		//same for decrement after and empty descending
	STMDB, STMFD,		//same for decrement before and full deescending
	STMIB, STMFA,		//same for increment before and full ascending
	//queue
	POP, PUSH,			//pop and push from/to the stack
	//misc
	CLREX,	//clear exclusive
	DGB,	//debug hint
	DMB,	//data memory barrier
	DSB,	//data synchronization barrier
	ISB,	//instruction synchronization barrier
	IT,		//if-then
	NOP,	//no operation
	PLD, PLDW,	//preload data
	PLI,	//preload instruction
	SETEND,	//set endianess
	SEV,	//set event
	SWP [[deprecated]], SWPB [[deprecated]], //swap byte
	WFE, WFI, //wait for event, wait for interruption
	YIELD,
	//exception handling
	SVC,	//supervisor call
	BKPT,	//breakpoint
	SMC,	//secure monitor call
	RFE,	//return from exception
	SUBS,	//subtract exception return
	LDM,	//load multiple exception return
	SRS,	//store return state
	//co-processor interaction
	CDP, CDP2, //initiate co-processor data operation
	MCR, MCR2, //copy register between the core and co-processors
	MCRR, MCRR2,
	MRC, MRC2,
	MRRC, MRRC2,
	LDC, LDC2, //load co-processor register values
	STC, STC2, //store co-processor register values
	//simd
	VLDM, VLDR, //vector load multiple, vector load register
	VSTM, VSTR,	//vector store multiple, vector store register
	//element and structure load store
	VLD1, VLD2, VLD3, VLD4, //load 1-, 2-, 3-, and 4-element structures
	VST1, VST2, VST3, VST4, //store 1-, 2-, 3-, and 4-element structures
	//simd fp register transfer
	VDUP,	//copy from core reg to all simd vector elements
	VMOV,	//copy from core to extension register
	VMRS, VMSR, //copy from simd fp to core, copy from core to simd fp
	//advanced simd
	VADD, VADDHN, 	//vector add, vector add returning high half
	VADDL, VADDW,	//vector add long, vector add wide
	VHADD, VHSUB,	//vector halving add, vector halving subtraction
	VPADAL,			//vector pairwise add and accumulative long
	VPADD, VPADDL,	//vector pairwise add, vector pairwise add long
	VRADDHN,		//vector rounding add and narrow return high half
	VRHADD,			//vector rounding halving add
	VRSUBHN,		//vector rounding subtraction and narrow returning high half
	VQADD,			//vector add saturating
	VQSUB,			//vector subtraction saturating
	VSUB, VSUBW, VSUBL,	//vector substraction, wide, long
	VSUBHN,			//vector substraction and narrow returning high half
	//bitwise simd
	VAND, VBIC, 		//vector and, vector bit clean w/ and complement
	VEOR, VORR,			//vector XOR, vector OR
	VBIT, VBIF,			//vector bitwise insert if true, insert if false
	VMOV, VMVN,			//vector bitwise move, vector bitwise not
	VORN,				//vector bitwise OR NOT
	VBSL,				//vector bitwise select
	//simd comparison
	

	

	










	XDRFT

};


// //logical
// #define AND 0x0
// #define EOR 0x1
// #define OR      /* pseudo ? */
// #define BIC /* bit clear */
// #define MVN /* move not */
// #define TST /* test bits */

// //arithmetic
// #define ADD 
// #define ADC /* add with carry */
// #define SUB 0x2
// #define SBC /* sub with carry */
// #define NEG /* negate */
// #define MUL /* multiply */
// #define CMP /* compare */
// #define CMN /* compare negative */


// //branches
// #define BEQ
// #define BNE
// #define BCS
// #define BCC
// #define BMI
// #define BPL
// #define BVS
// #define BVC
// #define BHI
// #define BLS
// #define BGE
// #define BLT
// #define BGT
// #define BLE
// //#define B
// #define BL
// #define BX /* optional state change */

// //shifts/rotate
// #define LSL /* logical shift left */
// #define LSR /* logical shift right */
// #define ASR /* arithmetic shift right */
// #define ROR /* rotate right */

// //moves
// #define MOV



/**
 * @brief This class models the ORCA-AT processor core. That core
 * implements the ARM's T32 instruction, A-Profile, instruction set
 * for ARMv7 architecure. The hereby represented core has has nothing
 * to do with any of ARM processors except the ISA. ORCA-AT is an 
 * abstract processor, made to illustrate the modeling capabilities 
 * of the ORCA-SIM framework. It is an "arm-like" core, and no ARM core
 * is being represented here. Besides, this project has no intention on
 * reproduce the behaviour of any ARM IP, neither emulate them. 
 * 
 * For more information on the implemented ISA, 
 * visit ARM's official documentation. 
 * https://developer.arm.com/architectures/cpu-architecture/r-profile/docs
 * https://developer.arm.com/docs/ddi0406/latest
 * 
 * 
 */
class TOrcaAT : TProcessorBase<uint32_t>{

    private:
        int32_t _registers[37]; /* all registers' array */
		
        int32_t _fetch();
        int32_t _decode();
        int32_t _execute();

    public:
		void Reset();
		TOrcaAT(string name);
		~TOrcaAT();
        SimulationTime Run();
};
#endif /* _TORCAAT_H */

