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
#include <THellfireProcessor.h>

void THellfireProcessor::dumpregs(risc_v_state *s){
	int32_t i;
	
	for (i = 0; i < 32; i+=4){
		printf("r%02d [%08x] r%02d [%08x] r%02d [%08x] r%02d [%08x]\n", \
		i, s->r[i], i+1, s->r[i+1], i+2, s->r[i+2], i+3, s->r[i+3]);
	}
	printf("pc: %08x\n", s->pc);
	printf("\n");
}

void THellfireProcessor::bp(risc_v_state *s, uint32_t ir){
	printf("pc: %08x, ir: %08x\n", s->pc, ir);
	dumpregs(s);
	getchar();
}

int32_t THellfireProcessor::mem_fetch(risc_v_state *s, uint32_t address){
	
	uint32_t data;
	s->sram->Read(address, (int8_t*)&data, 4); //4 x sizeof(uint8_t)
		
	return data;
}

int32_t THellfireProcessor::mem_read(risc_v_state *s, int32_t size, uint32_t address){
	
	uint32_t data;

	switch(address){
		case IRQ_VECTOR:	return s->vector;
		case IRQ_CAUSE:		return s->cause | 0x0080 | 0x0040;
		case IRQ_MASK:		return s->mask;
		case IRQ_STATUS:	return s->status;
		case IRQ_EPC:		return s->epc;
		case COUNTER:		return s->counter;
		case COMPARE:		return s->compare;
		case COMPARE2:		return s->compare2;
		case UART_READ:		return getchar();
		case UART_DIVISOR:	return 0;
	}
	
	//control cons
	if(address == s->comm_ack->GetAddr())    return s->comm_ack->Read();
	if(address == s->comm_intr->GetAddr())   return s->comm_intr->Read();
	if(address == s->comm_start->GetAddr())  return s->comm_start->Read();
	
	//self-id
	if(address == s->comm_id->GetAddr())     return s->comm_id->Read();
	
	#ifdef MEMORY_ENABLE_COUNTERS
	if(address == s->sram->GetCommCounterStore()->GetAddr()) return s->sram->GetCommCounterStore()->Read();
	if(address == s->sram->GetCommCounterLoad()->GetAddr())  return s->sram->GetCommCounterLoad()->Read();
	if(address == s->mem1->GetCommCounterStore()->GetAddr()) return s->mem1->GetCommCounterStore()->Read();
	if(address == s->mem1->GetCommCounterLoad()->GetAddr())  return s->mem1->GetCommCounterLoad()->Read();
	if(address == s->mem2->GetCommCounterStore()->GetAddr()) return s->mem2->GetCommCounterStore()->Read();
	if(address == s->mem2->GetCommCounterLoad()->GetAddr())  return s->mem2->GetCommCounterLoad()->Read();
	#endif /* MEMORY_ENABLE_COUNTERS */
	
	#ifdef HFRISCV_ENABLE_COUNTERS
	if(address == this->GetCommCounterArith()->GetAddr())     return this->GetCommCounterArith()->Read();
	if(address == this->GetCommCounterLogical()->GetAddr())   return this->GetCommCounterLogical()->Read();
	if(address == this->GetCommCounterShift()->GetAddr())     return this->GetCommCounterShift()->Read();
	if(address == this->GetCommCounterBranches()->GetAddr())  return this->GetCommCounterBranches()->Read();
	if(address == this->GetCommCounterJumps()->GetAddr())     return this->GetCommCounterJumps()->Read();
	if(address == this->GetCommCounterLoadStore()->GetAddr()) return this->GetCommCounterLoadStore()->Read();
	#endif /* HFRISCV_ENABLE_COUNTERS */
		
	#ifdef ROUTER_ENABLE_COUNTERS
	if(address == _router->GetCommCounterActive()->GetAddr()) return _router->GetCommCounterActive()->Read();
	#endif /* ROUTER_ENABLE_COUNTERS */
		
	UMemory* sel_mem = nullptr;
	
	//memread to mem0
	if(address >= s->sram->GetBase() && address <= s->sram->GetLastAddr()){
		sel_mem = s->sram;
	}else 
		
	//memread to mem1 	
	if(address >= s->mem1->GetBase() && address <= s->mem1->GetLastAddr()){
		sel_mem = s->mem1;
	}
		
	#ifndef HFRISCV_READ_ADDRESS_CHECKING
	if(sel_mem == nullptr){
		dumpregs(s);
		stringstream ss;
		ss << this->GetName() << ": unable to read from unmapped memory space 0x" << std::hex << address << ".";
		throw std::runtime_error(ss.str());
	}
	#endif
	
	switch(size){
		case 4:
			if(address & 3){
				std::string err_msg = this->GetName() + ": unaligned access (load word) pc=0x" 
					+ std::to_string(s->pc) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				sel_mem->Read(address, (int8_t*)&data, 4); //4 x sizeof(uint8_t)
			}
			break;
		case 2:
			if(address & 1){
				std::string err_msg = this->GetName() + ": unaligned access (load halfword) pc=0x" 
					+ std::to_string(s->pc) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				int16_t value;
				sel_mem->Read(address, (int8_t*)&value, 2); //2 x sizeof(uint8_t)
				data = value;
			}
			break;
		case 1:
			int8_t value;
			sel_mem->Read(address, &value, 1); //1 x sizeof(uint8_t)
			data = value;
			break;
		default:
			std::string err_msg = this->GetName() + ": unknown02";
			throw std::runtime_error(err_msg);
	}

	return data;
}

void THellfireProcessor::mem_write(risc_v_state *s, int32_t size, uint32_t address, uint32_t value){
	
	switch(address){

		case IRQ_STATUS:{
			if (value == 0){ 
				s->status = 0; 
				for (int i = 0; i < 4; i++) 
					s->status_dly[i] = 0; 
				}else{ 
					s->status_dly[3] = value; 
			}
			return;
		}
		case IRQ_VECTOR:	s->vector = value; return;
		case IRQ_CAUSE:		s->cause = value; return;
		case IRQ_MASK:		s->mask = value; return;
		case IRQ_EPC:		s->epc = value; return;
		case COUNTER:		s->counter = value; return;
		case COMPARE:		s->compare = value; s->cause &= 0xffef; return;
		case COMPARE2:		s->compare2 = value; s->cause &= 0xffdf; return;

		case DEBUG_ADDR: output_debug << (int8_t)(value & 0xff) << std::flush; return;
		case UART_WRITE: output_uart << (int8_t)(value & 0xff) << std::flush; return;
		case UART_DIVISOR: return;

		case EXIT_TRAP:
			std::cout << this->GetName() <<": exit trap triggered! (" << std::dec << s->cycles << " cycles)" << std::endl;
			output_debug.close();
			output_uart.close();
			return;
	}
	
	//comms
	if(address == s->comm_ack->GetAddr()){   s->comm_ack->Write(value);   return; }
	if(address == s->comm_start->GetAddr()){ s->comm_start->Write(value); return; }
	
	#ifdef MEMORY_ENABLE_COUNTERS
	if(address == s->sram->GetCommCounterStore()->GetAddr()){s->sram->GetCommCounterStore()->Write(0); return;}
	if(address == s->sram->GetCommCounterLoad()->GetAddr()) {s->sram->GetCommCounterLoad()->Write(0);  return;}
	if(address == s->mem1->GetCommCounterStore()->GetAddr()){s->mem1->GetCommCounterStore()->Write(0); return;}
	if(address == s->mem1->GetCommCounterLoad()->GetAddr()) {s->mem1->GetCommCounterLoad()->Write(0);  return;}
	if(address == s->mem2->GetCommCounterStore()->GetAddr()){s->mem2->GetCommCounterStore()->Write(0); return;}
	if(address == s->mem2->GetCommCounterLoad()->GetAddr()) {s->mem2->GetCommCounterLoad()->Write(0);  return;}
	#endif /* OPT_MEMORY_DISABLE_COUNTERS */
	
	#ifdef MEMORY_ENABLE_COUNTERS
	if(address == this->GetCommCounterArith()->GetAddr())     {this->GetCommCounterArith()->Write(0);     return;}
	if(address == this->GetCommCounterLogical()->GetAddr())   {this->GetCommCounterLogical()->Write(0);   return;}
	if(address == this->GetCommCounterShift()->GetAddr())     {this->GetCommCounterShift()->Write(0);     return;}
	if(address == this->GetCommCounterBranches()->GetAddr())  {this->GetCommCounterBranches()->Write(0);  return;}
	if(address == this->GetCommCounterJumps()->GetAddr())     {this->GetCommCounterJumps()->Write(0);     return;}
	if(address == this->GetCommCounterLoadStore()->GetAddr()) {this->GetCommCounterLoadStore()->Write(0); return;}
	#endif /* OPT_HFRISC_DISABLE_COUNTERS */
	
	#ifdef ROUTER_ENABLE_COUNTERS
	if(address == _router->GetCommCounterActive()->GetAddr()) {_router->GetCommCounterActive()->Write(0); return;}
	#endif /* ROUTER_ENABLE_COUNTERS */
	
	UMemory* sel_mem = nullptr;
	
	//memwrite to sram
	if(address >= s->sram->GetBase() && address <= s->sram->GetLastAddr()){
		sel_mem = s->sram;
	}else 
	
	//memwrite to mem2 	
	if(address >= s->mem2->GetBase() && address <= s->mem2->GetLastAddr()){
		sel_mem = s->mem2;
	}
		
	#ifdef HFRISCV_WRITE_ADDRESS_CHECKING
	if(sel_mem == nullptr){
		dumpregs(s);
		stringstream ss;
			
		ss << this->GetName() << ": unable to write to unmapped memory space 0x" << std::hex << address << ".";
		
		throw std::runtime_error(ss.str());
	}
	#endif
	
	switch(size){
		case 4:
			if(address & 3){
				std::string err_msg = this->GetName() + ": unaligned access (store word) pc=0x" + std::to_string(s->pc) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				sel_mem->Write(address, (int8_t*)&value, size);
			}
			break;
		case 2:
			if(address & 1){
				std::string err_msg = this->GetName() + ": unaligned access (store halfword) pc=0x" + std::to_string(s->pc) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				uint16_t data = (uint16_t)value;
				sel_mem->Write(address, (int8_t*)&data, size);
			}
			break;
		case 1:{
			uint8_t data;
			data = (uint8_t)value;
			sel_mem->Write(address, (int8_t*)&data, size);
			break;
		}
		default:
			std::string err_msg = this->GetName() + ": unable to write to memory (unk02)";
			throw std::runtime_error(err_msg);
	}
	
	
}

#ifdef HFRISCV_ENABLE_COUNTERS

//Counters' getters
UComm<uint32_t>* THellfireProcessor::GetCommCounterArith(){
	return this->_counter_iarith;
}
UComm<uint32_t>* THellfireProcessor::GetCommCounterLogical(){
	return this->_counter_ilogical;
}
UComm<uint32_t>* THellfireProcessor::GetCommCounterShift(){
	return this->_counter_ishift;
}
UComm<uint32_t>* THellfireProcessor::GetCommCounterBranches(){
	return this->_counter_ibranches;
}
UComm<uint32_t>* THellfireProcessor::GetCommCounterJumps(){
	return this->_counter_ijumps;
}
UComm<uint32_t>* THellfireProcessor::GetCommCounterLoadStore(){
	return this->_counter_iloadstore;
}

/**
 * Initialize Counters
 * memory-mapped address of counters must be informed
 */

void THellfireProcessor::InitCounters(
		uint32_t arith_counter_addr, 
		uint32_t logical_counter_addr,
		uint32_t shift_counter_addr,
		uint32_t branches_counter_addr,
		uint32_t jumps_counter_addr, 
		uint32_t loadstore_counter_addr){
		
	this->_counter_iarith     = new UComm<uint32_t>(GetName() + ".counters.iarith",     0, arith_counter_addr);
	this->_counter_ilogical   = new UComm<uint32_t>(GetName() + ".counters.ilogical",   0, logical_counter_addr);
	this->_counter_ishift     = new UComm<uint32_t>(GetName() + ".counters.ishift",     0, shift_counter_addr);
	this->_counter_ibranches  = new UComm<uint32_t>(GetName() + ".counters.ibranches",  0, branches_counter_addr);
	this->_counter_ijumps     = new UComm<uint32_t>(GetName() + ".counters.ijumps",     0, jumps_counter_addr);
	this->_counter_iloadstore = new UComm<uint32_t>(GetName() + ".counters.iloadstore", 0, loadstore_counter_addr);
}

/**
 * Update Counters
 * Counters are incremented by one every time some instruction
 * is executed. Please note that NOP and MOVE instructions are
 * ignored as riscv32i does not generate them.
*/
void THellfireProcessor::UpdateCounters(int opcode, int funct3){

	switch(opcode){
		
		case 0x37: //LUI
		case 0x17: //ALUI
		case 0x6f: //JAL
		case 0x67: //JALR
			_counter_ijumps->Inc(1);
			break;
		
		case 0x63: //branches
			_counter_ibranches->Inc(1);
			break;
			
		case 0x3:  //loads
		case 0x23: //stores
			_counter_iloadstore->Inc(1);
			break;

		//type R
		case 0x13:
			switch(funct3){
				case 0x0: //addi, subi
					_counter_iarith->Inc(1);
					break;
					
				case 0x4: //xori
				case 0x6: //ori
				case 0x7: //andi
					_counter_ilogical->Inc(1);
					break;

				default: //shifts i
					_counter_ishift->Inc(1);
					break;
			}
			break;
			
		case 0x33:
			switch(funct3){
				case 0x0: //add, sub
					_counter_iarith->Inc(1);
					break;
				case 0x4: //xor
				case 0x6: //or
				case 0x7: //and
					_counter_ilogical->Inc(1);
					break;
				default: //all shifts
					_counter_ishift->Inc(1);
			}
			break;
			
		default:
			break;
	}
	
}
#endif /* HFRISCV_ENABLE_COUNTERS */


unsigned long long THellfireProcessor::Run(){
		
	uint32_t inst, i;
	uint32_t opcode, rd, rs1, rs2, funct3, funct7, imm_i, imm_s, imm_sb, imm_u, imm_uj;
	int32_t *r = s->r;
	uint32_t *u = (uint32_t *)s->r;
	uint32_t ptr_l, ptr_s;
	
	if (s->status && (s->cause & s->mask)){
		s->epc = s->pc_next;
		s->pc = s->vector;
		s->pc_next = s->vector + 4;
		s->status = 0;
		for (i = 0; i < 4; i++)
			s->status_dly[i] = 0;
	}

	inst = mem_fetch(s, s->pc);

	opcode = inst & 0x7f;
	
	rd = (inst >> 7) & 0x1f;
	rs1 = (inst >> 15) & 0x1f;
	rs2 = (inst >> 20) & 0x1f;
	funct3 = (inst >> 12) & 0x7;
	funct7 = (inst >> 25) & 0x7f;
	imm_i = (inst & 0xfff00000) >> 20;
	imm_s = ((inst & 0xf80) >> 7) | ((inst & 0xfe000000) >> 20);
	imm_sb = ((inst & 0xf00) >> 7) | ((inst & 0x7e000000) >> 20) | ((inst & 0x80) << 4) | ((inst & 0x80000000) >> 19);
	imm_u = inst & 0xfffff000;
	imm_uj = ((inst & 0x7fe00000) >> 20) | ((inst & 0x100000) >> 9) | (inst & 0xff000) | ((inst & 0x80000000) >> 11); 
	if (inst & 0x80000000){
		imm_i |= 0xfffff000;
		imm_s |= 0xfffff000;
		imm_sb |= 0xffffe000;
		imm_uj |= 0xffe00000;
	}
	ptr_l = r[rs1] + (int32_t)imm_i;
	ptr_s = r[rs1] + (int32_t)imm_s;
	r[0] = 0;

	switch(opcode){
		case 0x37: r[rd] = imm_u; break;										/* LUI */
		case 0x17: r[rd] = s->pc + imm_u; break;									/* AUIPC */
		
		case 0x6f: r[rd] = s->pc_next; s->pc_next = s->pc + imm_uj; break;				  /* JAL */
		case 0x67: r[rd] = s->pc_next; s->pc_next = (r[rs1] + imm_i) & 0xfffffffe; break; /* JALR */
		case 0x63:
			switch(funct3){
				case 0x0: if (r[rs1] == r[rs2]){ s->pc_next = s->pc + imm_sb; } break;				/* BEQ */
				case 0x1: if (r[rs1] != r[rs2]){ s->pc_next = s->pc + imm_sb; } break;				/* BNE */
				case 0x4: if (r[rs1] < r[rs2]){ s->pc_next = s->pc + imm_sb; } break;				/* BLT */
				case 0x5: if (r[rs1] >= r[rs2]){ s->pc_next = s->pc + imm_sb; } break;				/* BGE */
				case 0x6: if (u[rs1] < u[rs2]){ s->pc_next = s->pc + imm_sb; } break;				/* BLTU */
				case 0x7: if (u[rs1] >= u[rs2]){ s->pc_next = s->pc + imm_sb; } break;				/* BGEU */
				default: goto fail;
			}
			break;
		case 0x3:
			switch(funct3){
				case 0x0: r[rd] = (int8_t)mem_read(s,1,ptr_l); break;						/* LB */
				case 0x1: r[rd] = (int16_t)mem_read(s,2,ptr_l); break;						/* LH */
				case 0x2: r[rd] = mem_read(s,4,ptr_l); break;							/* LW */
				case 0x4: r[rd] = (uint8_t)mem_read(s,1,ptr_l); break;						/* LBU */
				case 0x5: r[rd] = (uint16_t)mem_read(s,2,ptr_l); break;						/* LHU */
				default: goto fail;
			}
			break;
		case 0x23:
			switch(funct3){
				case 0x0: mem_write(s,1,ptr_s,r[rs2]); break;							/* SB */
				case 0x1: mem_write(s,2,ptr_s,r[rs2]); break;							/* SH */
				case 0x2: mem_write(s,4,ptr_s,r[rs2]); break;							/* SW */
				default: goto fail;
			}
			break;
		case 0x13:
			switch(funct3){
				case 0x0: r[rd] = r[rs1] + (int32_t)imm_i; break;						/* ADDI */
				case 0x2: r[rd] = r[rs1] < (int32_t)imm_i; break;		 				/* SLTI */
				case 0x3: r[rd] = u[rs1] < (uint32_t)imm_i; break;						/* SLTIU */
				case 0x4: r[rd] = r[rs1] ^ (int32_t)imm_i; break;						/* XORI */
				case 0x6: r[rd] = r[rs1] | (int32_t)imm_i; break;						/* ORI */
				case 0x7: r[rd] = r[rs1] & (int32_t)imm_i; break;						/* ANDI */
				case 0x1: r[rd] = u[rs1] << (rs2 & 0x3f); break;						/* SLLI */
				case 0x5:
					switch(funct7){
						case 0x0: r[rd] = u[rs1] >> (rs2 & 0x3f); break;				/* SRLI */
						case 0x20: r[rd] = r[rs1] >> (rs2 & 0x3f); break;				/* SRAI */
						default: goto fail;
					}
					break;
				default: goto fail;
			}
			break;
		case 0x33:
			switch(funct3){
				case 0x0:
					switch(funct7){
						case 0x0: r[rd] = r[rs1] + r[rs2]; break;					/* ADD */
						case 0x20: r[rd] = r[rs1] - r[rs2]; break;					/* SUB */
						default: goto fail;
					}
					break;
				case 0x1: r[rd] = r[rs1] << r[rs2]; break;							/* SLL */
				case 0x2: r[rd] = r[rs1] < r[rs2]; break;		 					/* SLT */
				case 0x3: r[rd] = u[rs1] < u[rs2]; break;		 					/* SLTU */
				case 0x4: r[rd] = r[rs1] ^ r[rs2]; break;							/* XOR */
				case 0x5:
					switch(funct7){
						case 0x0: r[rd] = u[rs1] >> u[rs2]; break;					/* SRL */
						case 0x20: r[rd] = r[rs1] >> r[rs2]; break;					/* SRA */
						default: goto fail;
					}
					break;
				case 0x6: r[rd] = r[rs1] | r[rs2]; break;							/* OR */
				case 0x7: r[rd] = r[rs1] & r[rs2]; break;							/* AND */
				default: goto fail;
			}
			break;
		default:
fail:
			stringstream ss;
			ss << this->GetName() << ":invalid opcode (pc=0x" << std::hex << s->pc;
			ss << " opcode=0x" << std::hex << inst << ")";
			
			s->sram->Dump(s->pc, 500);
			
			throw std::runtime_error(ss.str());
			break;
	}
	
	_last_pc = s->pc;
	s->pc = s->pc_next;
	s->pc_next = s->pc_next + 4;
	s->status = s->status_dly[0];
	for (i = 0; i < 3; i++)
		s->status_dly[i] = s->status_dly[i+1];
	
	s->cycles++;
	s->counter++;
	
	if ((s->compare2 & 0xffffff) == (s->counter & 0xffffff)) s->cause |= 0x20;     /*IRQ_COMPARE2*/
	if (s->compare == s->counter) s->cause |= 0x10;                                /*IRQ_COMPARE*/
	if (!(s->counter & 0x10000)) s->cause |= 0x8; else s->cause &= 0xfffffff7;     /*IRQ_COUNTER2_NOT*/
	if (s->counter & 0x10000) s->cause |= 0x4; else s->cause &= 0xfffffffb;        /*IRQ_COUNTER2*/
	if (!(s->counter & 0x40000)) s->cause |= 0x2; else s->cause &= 0xfffffffd;     /*IRQ_COUNTER_NOT*/
	if (s->counter & 0x40000) s->cause |= 0x1; else s->cause &= 0xfffffffe;        /*IRQ_COUNTER*/
	
	if (s->comm_intr->Read() == 0x1) s->cause |= 0x100; else s->cause &= 0xfffffeff; /*NOC*/

	#ifdef HFRISCV_ENABLE_COUNTERS
	this->UpdateCounters(opcode, funct3);
	#endif /*HFRISCV_ENABLE_COUNTERS*/
	
	//Takes three cycles per instruction, except for those of 
	//memory I/O. In the later case. Since we simulate the pipeline
	//by executing one instruction per cycle (starting from the 3th cycle),
	//we add 1 cycle to simulate I/O delay.
	return (opcode == 0x23 || opcode == 0x3) ? 2 : 1;
}

risc_v_state THellfireProcessor::GetState(){
	return *s;
}

/**
 * @brief Configures main memory module.
 * @param m A pointer to a UMemory object*/
void THellfireProcessor::SetMem0(UMemory* m){
	
	s->pc = m->GetBase();
	s->pc_next = s->pc + 4;

	s->sram = m;
}

void THellfireProcessor::SetMem1(UMemory* m){
	s->mem1 = m;
}

void THellfireProcessor::SetMem2(UMemory* m){
	s->mem2 = m;
}

//setters for comms
void THellfireProcessor::SetCommAck(UComm<int8_t>* comm){
	s->comm_ack = comm;
}

void THellfireProcessor::SetCommId(UComm<int32_t>* comm){
	s->comm_id = comm;
}

void THellfireProcessor::SetCommIntr(UComm<int8_t>* comm){
	s->comm_intr = comm;
}

void THellfireProcessor::SetCommStart(UComm<int8_t>* comm){
	s->comm_start = comm;
}

/**
TODO: remove router uinstance from the inside the processor core
*/
void THellfireProcessor::SetRouter(TRouter* r){
	_router = r;
}

THellfireProcessor::THellfireProcessor(string name) : TimedModel(name) {

	s = &context;
	memset(s, 0, sizeof(risc_v_state));
	
	s->vector = 0;
	s->cause = 0;
	s->mask = 0;
	s->status = 0;
	
	for (i = 0; i < 4; i++)
		s->status_dly[i] = 0;
		
	s->epc = 0;
	s->counter = 0;
	s->compare = 0;
	s->compare2 = 0;
	s->cycles = 0;
		
	output_debug.open("logs/" + this->GetName() + "_debug.log", std::ofstream::out | std::ofstream::trunc);
	output_uart.open("logs/" + this->GetName() + "_uart.log", std::ofstream::out | std::ofstream::trunc);

}

//TODO: clear allocated memory if any
THellfireProcessor::~THellfireProcessor(){
	
	#ifdef HFRISCV_ENABLE_COUNTERS
	delete _counter_iarith;
	delete _counter_ilogical;
	delete _counter_ishift;
	delete _counter_ibranches;
	delete _counter_ijumps;
	delete _counter_iloadstore;
	#endif
}

void THellfireProcessor::Reset(){
    //TODO: to be implemented
    return;
}
