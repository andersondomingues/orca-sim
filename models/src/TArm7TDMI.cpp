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
#include <cstdlib>
#include <sstream>
		
#include "TArm7TDMI.h"
		

void TArm7TDMI::Reset(){
	return;
}

TArm7TDMI::TArm7TDMI(string name) : TimedModel(name) {
}

TArm7TDMI::~TArm7TDMI(){

}

long long unsigned int TArm7TDMI::Run(){

	//fetch 
	uint32_t instruction = _mem->Read(PC, &instruction, sizeof(uint32_t);

	if(thumb){
	
		//BX - branch and exchange
		//[31:28][0001][0010][1111][1111][1111][0001][03:00]
		// cond                                        RN
		case BX: //takes 2S + 1N
			if(RN[0] == 1) //switch to thumb
			else //switch to arm
			break;

		//B - branch
		//BL - branch and link
		//[31:28][101L][                 23:0              ]
		// cond  link-bit               offset
		case B:  //takes 2S + 1N
		case BL: //takes 2S + 1N
			if(L == 1) //bl
			else //b
			
		//data processing 
		//[31:28][00I][24:21][S][19:16][15:12][   11:00    ]
		// cond  imm  opcode set  RN     RD      operand2
		case DP:
		
			if(I == 0) //operand 2 is a register
				
			else       //operand 2 is immediate
				 
		
			switch(opcode){
				case 0x0: RD = RN & OP2;        break; //AND
				case 0x1: RD = RN <EOR> OP2;    break; //EOR (XOR)
				case 0x2: RD = RN - OP2;        break; //SUB
				case 0x3: RD = OP2 - RN;        break; //RSB
				case 0x4: RD = RN + OP2;        break; //ADD
				case 0x5: RD = RN + OP2 + C;    break; //ADC, C is for carry
				case 0x6: RD = RN - OP2 + C -1; break; //SBC
				case 0x7: RD = OP2 - RN + C -1; break; //RSC
				case 0x8: RD = CHECK_COND(COND, RN and OP2); break; //TST
				case 0x9: RD = CHECK_COND(COND, RN xor OP2); break; //TEQ
				case 0xA: RD = CHECK_COND(COND, RN + OP2);   break; //CMP
				case 0xB: RD = CHECK_COND(COND, RN - OP2);   break; //CMN
				case 0xC: RD = RN | OP2;  break; //ORR (OR)
				case 0xD: RD = OP2;       break; //MOV
				case 0xE: RD = RN & !OP2; break; //BIC
				case 0xF: RD = !OP2       break; //MVN
				default:
					//unreachable 
					break;
			}
			break;
		
		case MUL: //multiply
		case MLA: //multiply accumulate
			break;
			
		case SWI: //software interrupt
			
		
			break;
	}

	



	return 1;
}

uint32_t CHECK_COND(COND, uint32_t OP1, uint32_t OP2){

switch(COND)
	case EQ: return OP1 == OP2; //equals
	case NE: return OP1 != OP2, //not equals
	case CS: return OP1 >= OP2; //unsigned higher or same
	case CC: return OP1 < OP2;  //unsigned lower
	case MI: return OP1 < 0;    //negative
	case PL: return OP1 >= 0;   //positive or zero
	//TODO case VS: return; // V set, overflow
	//TODO case VC: return; // V clear, not overflow
	case HI: return OP1 > OP2;  //unsigned higher
	case LS: return OP1 <= OP2; //unsigned lower or same
	case GE: return OP1 >= OP2; //greater of equals
	case LT: return OP1 < OP2;  //less than
	case GT: return OP1 > OP2;  //greater than
	case LE: return OP1 <= OP2; //less than or equal
	//TODO case AL:return; // (ignored), always
	default:
		//unreachable
		break;



}
