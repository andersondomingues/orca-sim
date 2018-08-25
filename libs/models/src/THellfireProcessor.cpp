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
#include <THellfireProcessor.h>
#include <cstdlib>

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
	s->mem->Read(address, (int8_t*)&data, 4); //4 x sizeof(uint8_t)
		
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
		
		//dmni read-only space
		case DMNI_SEND_ACTIVE: 
			output_debug << "mem_read from DMNI_SEND_ACTIVE" << std::flush;
			return _dmni->GetSendActive();
		case DMNI_RECEIVE_ACTIVE: 
			output_debug << "mem_read from DMNI_RECEIVE_ACTIVE" << std::flush;
			return _dmni->GetReceiveActive();		
	}

	//ptr = (uint32_t *)(s->mem + (address % MEM_SIZE));
	
	#ifndef NOGUARDS
	if(address < SRAM_BASE){
		dumpregs(s);
		throw std::runtime_error(this->GetName() + ": unable to read from unmapped memory memory space (lower than sram_base) " + std::to_string(address) + ".");
	}
	if(address > SRAM_BASE + MEM_SIZE){
		dumpregs(s);
		throw std::runtime_error(this->GetName() + ": unable to read from unmapped memory memory space (greater than sram_base + mem_size) " + std::to_string(address) + ".");
	}
	#endif	
	
	switch(size){
		case 4:
			if(address & 3){
				std::string err_msg = this->GetName() + ": unaligned access (load word) pc=0x" + std::to_string(s->pc) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				/*value = *(int32_t *)ptr; */
				s->mem->Read(address, (int8_t*)&data, 4); //4 x sizeof(uint8_t)
			}
			break;
		case 2:
			if(address & 1){
				std::string err_msg = this->GetName() + ": unaligned access (load halfword) pc=0x" + std::to_string(s->pc) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				/*value = *(int16_t *)ptr;*/
				int16_t value;
				s->mem->Read(address, (int8_t*)&value, 2); //2 x sizeof(uint8_t)
				data = value;
			}
			break;
		case 1:
			/*value = *(int8_t *)ptr;*/
			int8_t value;
			s->mem->Read(address, &value, 1); //1 x sizeof(uint8_t)
			data = value;
			break;
		default:
			std::string err_msg = this->GetName() + ": unknown02";
			throw std::runtime_error(err_msg);
	}

	/*return(value);*/
	return data;
}

void THellfireProcessor::mem_write(risc_v_state *s, int32_t size, uint32_t address, uint32_t value){

	/*uint32_t *ptr;*/

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

		case EXIT_TRAP:
			std::cout << this->GetName() <<": exit trap triggered! (" << s->cycles << " cycles)" << std::endl;
			_disabled = true;
			output_debug.close();
			output_uart.close();
			return;
		case DEBUG_ADDR:
			output_debug << (int8_t)(value & 0xff) << std::flush;
			return;
		case UART_WRITE:
			output_uart << (int8_t)(value & 0xff) << std::flush;
			return;
		case UART_DIVISOR:
			return;

		//dmni write-only space
		case DMNI_SIZE: 
			output_debug << "mem_write to DMNI_SIZE" << std::flush;
			s->dmni_size = value;
			return;
		case DMNI_OP: 
			output_debug << "mem_write to DMNI_OP" << std::flush;
			s->dmni_op = value;
			return;
		case DMNI_ADDRESS: 
			output_debug << "mem_write to DMNI_ADDRESS" << std::flush;
			s->dmni_addr = value;
			return;
		case DMNI_START:{ 
			output_debug << "mem_write to DMNI_START" << std::flush;
			switch(s->dmni_op){
				case DMNI_WRITE:
					_dmni->CopyTo(s->dmni_addr, s->dmni_size);
					break;
				case DMNI_READ:
					_dmni->CopyFrom(s->dmni_addr, s->dmni_size);
					break;
			}
			return;
		}	
			
		//dmni read-only space
		case DMNI_SEND_ACTIVE: 
			output_debug << "mem_write to DMNI_START" << std::flush;
			throw std::runtime_error(this->GetName() + ": unable to write to write-protected address (DMNI_SEND_ACTIVE)");
			return;
		case DMNI_RECEIVE_ACTIVE: 
			output_debug << "mem_write to DMNI_START" << std::flush;
			throw std::runtime_error(this->GetName() + ": unable to write to write-protected address (DMNI_RECEIVE_ACTIVE)");
			return;
	}
	
	#ifndef NOGUARDS
	if(address < SRAM_BASE){
		dumpregs(s);
		throw std::runtime_error(this->GetName() + ": unable to write to unmapped memory memory space (lower than sram_base) " + std::to_string(address) + ".");
	}
	if(address > SRAM_BASE + MEM_SIZE){
		dumpregs(s);
		throw std::runtime_error(this->GetName() + ": unable to write to unmapped memory memory space (greater than sram_base + mem_size) " + std::to_string(address) + ".");
	}
	#endif
		
	switch(size){
		case 4:
			if(address & 3){
				std::string err_msg = this->GetName() + ": unaligned access (store word) pc=0x" + std::to_string(s->pc) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				/*  *(int32_t *)ptr = value;*/
				//int32_t val = value;
				s->mem->Write(address, (int8_t*)&value, size);
			}
			break;
		case 2:
			if(address & 1){
				std::string err_msg = this->GetName() + ": unaligned access (store halfword) pc=0x" + std::to_string(s->pc) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				/*  *(int16_t *)ptr = (uint16_t)value;*/
				uint16_t data = (uint16_t)value;
				s->mem->Write(address, (int8_t*)&data, size);
			}
			break;
		case 1:
			/*  *(int8_t *)ptr = (uint8_t)value;*/
			uint8_t data;
			data = (uint8_t)value;
			s->mem->Write(address, (int8_t*)&data, size);
			break;
			
		default:
			std::string err_msg = this->GetName() + ": unknown01";
			throw std::runtime_error(err_msg);
	}
}


unsigned long long THellfireProcessor::Run(){

	
	return this->cycle(this->s);
}

unsigned long long THellfireProcessor::cycle(risc_v_state *s){
		
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
		case 0x6f: r[rd] = s->pc_next; s->pc_next = s->pc + imm_uj; break;						/* JAL */
		case 0x67: r[rd] = s->pc_next; s->pc_next = (r[rs1] + imm_i) & 0xfffffffe; break;				/* JALR */
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
		default: goto fail;
	}
	
	s->pc = s->pc_next;
	s->pc_next = s->pc_next + 4;
	s->status = s->status_dly[0];
	for (i = 0; i < 3; i++)
		s->status_dly[i] = s->status_dly[i+1];
	
	s->cycles++;
	s->counter++;
	
	if ((s->compare2 & 0xffffff) == (s->counter & 0xffffff)) s->cause |= 0x20;     /*IRQ_COMPARE2*/
	if (s->compare == s->counter) s->cause |= 0x10;                                /*IRQ_COMPARE*/
	if (!(s->counter & 0x10000)) s->cause |= 0x8; else s->cause &= 0xfff7;			/*IRQ_COUNTER2_NOT*/
	if (s->counter & 0x10000) s->cause |= 0x4; else s->cause &= 0xfffb;			/*IRQ_COUNTER2*/
	if (!(s->counter & 0x40000)) s->cause |= 0x2; else s->cause &= 0xfffd;			/*IRQ_COUNTER_NOT*/
	if (s->counter & 0x40000) s->cause |= 0x1; else s->cause &= 0xfffe;			/*IRQ_COUNTER*/
	
	
	//returns 4 of Store or Load, else returns 3
	return (opcode == 0x23 || opcode == 0x3) ? 4 : 3;
	
fail:
	std::string err_msg = this->GetName() + ":invalid opcode (pc=0x" + std::to_string(s->pc) + " opcode=0x" + std::to_string(inst) + ")";
	throw std::runtime_error(err_msg);
}

THellfireProcessor::THellfireProcessor(string name, UMemory* mptr, TDmni* dmni, uint32_t size, uint32_t base) : TimedModel(name) {

	s = &context;
	memset(s, 0, sizeof(risc_v_state));
	
	s->pc = base;
	s->pc_next = s->pc + 4;

	s->mem = mptr;

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
	
	_dmni = dmni;
	_disabled = false;
	
	output_debug.open("logs/" + this->GetName() + "_debug.log");
	output_uart.open("logs/" + this->GetName() + "_uart.log");
}

//TODO: clear allocated memory if any
THellfireProcessor::~THellfireProcessor(){}

/**
 * @brief Processor reset.*/
void THellfireProcessor::Reset(){
    //TODO: to be implemented
    return;
}
