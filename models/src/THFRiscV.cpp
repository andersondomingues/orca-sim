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
#include <cstdlib>
#include <sstream>
#include <chrono>

#include <THFRiscV.h>
#include <USignal.h>

#include "sys/time.h"

#define RISCV_INVALID_OPCODE 0x0

//api access shorthands (must be undef at the end of the file)
#define PC GetState()->pc
#define PC_NEXT GetState()->pc_next
#define R GetState()->regs

void THFRiscV::dumpregs(){

	for (uint32_t i = 0; i < 32; i += 4){
		printf("r%02d [%08x] r%02d [%08x] r%02d [%08x] r%02d [%08x]\n", \
		i, R[i], i+1, R[i+1], i+2, R[i+2], i+3, R[i+3]);
	}

	printf("pc: %08x\n\n", PC);
}

void THFRiscV::bp(risc_v_state *s, uint32_t ir){

	printf("breakpoint reached!\n");
	printf("pc: %08x, ir: %08x\n", PC, ir);
	printf("irq_status: %08x, irq_cause: %08x, irq_mask: %08x\n", s->status, s->cause, s->mask);
	dumpregs();

	stringstream ss;
	ss << "breakpoints/bp.0x" << std::hex << s->counter << std::dec << ".bin";
	
	switch(ir){
		case 0x0: std::cout << "RISCV_INVALID_OPCODE"; break;
		default:  std::cout << "UNKNOWN"; break;
	}
	
	std::cout << std::endl;

	GetMemory()->SaveBin(ss.str(), GetMemory()->GetBase(), GetMemory()->GetSize());
}

/**
 * @brief Reads data from the memory
 * @param s Current state of the core
 * @param size Size of data to be read. Must be 32, 16, or 8.
 * @param address Starting address to read from
 * @return Data read
 */
int32_t THFRiscV::mem_read(risc_v_state *s, int32_t size, uint32_t address){

	uint32_t data;
	
	//Check whether the address belongs to the main memory
	if(address <= GetMemory()->GetLastAddr() && address > GetMemory()->GetBase()){
		
		#ifdef HFRISCV_ENABLE_COUNTERS
		//update clock only when requested
		if(address == _counter_hosttime->GetAddress()){

			std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
			auto duration = now.time_since_epoch();
			auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
			
			_counter_hosttime->Write(millis);
		}
		#endif
		
		switch(size){
		case 4:
			if(address & 3){
				std::string err_msg = GetName() + ": unaligned access (load word) pc=0x" 
					+ std::to_string(PC) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				GetMemory()->Read(address, (int8_t*)&data, 4); //4 x sizeof(uint8_t)
			}
			break;
		case 2:
			if(address & 1){
				std::string err_msg = GetName() + ": unaligned access (load halfword) pc=0x" 
					+ std::to_string(PC) + " addr=0x" + std::to_string(address);
				throw std::runtime_error(err_msg);
			}else{
				int16_t value;
				GetMemory()->Read(address, (int8_t*)&value, 2); //2 x sizeof(uint8_t)
				data = value;
			}
			break;
		case 1:
			int8_t value;
			GetMemory()->Read(address, &value, 1); //1 x sizeof(uint8_t)
			data = value;
			break;
		default:
			std::string err_msg = GetName() + ": could not read from memory, invalid data size requested";
			throw std::runtime_error(err_msg);
		}
		
		return data;
		
	}else{
		
		//Address does not belong to any bank, check for special addresses
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
			
		//may the requested address fall in unmapped range, halt the simulation
		dumpregs();
		stringstream ss;
		ss << GetName() << ": unable to read from unmapped memory space 0x" << std::hex << address << ".";
		throw std::runtime_error(ss.str());
	}
}

USignal<uint8_t>* THFRiscV::GetSignalStall(){
	return _signal_stall;
}

USignal<uint8_t>* THFRiscV::GetSignalIntr(){
	return _signal_intr;
}
	
/**
 * @brief Reads data from memory
 * @param s The current state of the processor
 * @param size Size of data to be read. Must be 32, 16 or 8.
 * @param address Starting address of data
 * @param value Value to be written to the address
 */
void THFRiscV::mem_write(risc_v_state *s, int32_t size, uint32_t address, uint32_t value){

	//if the address belong to some memory range, write to it
	if(address <= GetMemory()->GetLastAddr()){

		switch(size){
			case 4:
				if(address & 3){
					stringstream ss;
					ss << GetName() << ": unaligned access (store word) pc=0x" 
					   << std::hex << PC << " addr=0x" << std::hex << address;
					throw std::runtime_error(ss.str());
				}else{
					GetMemory()->Write(address, (int8_t*)&value, size);
				}
				break;
			case 2:
				if(address & 1){
					std::string err_msg = GetName() 
						+ ": unaligned access (store halfword) pc=0x" 
						+ std::to_string(PC) + " addr=0x" 
						+ std::to_string(address);
					throw std::runtime_error(err_msg);
				}else{
					uint16_t data = (uint16_t)value;
					GetMemory()->Write(address, (int8_t*)&data, size);
				}
				break;
			case 1:{
				uint8_t data;
				data = (uint8_t)value;
				GetMemory()->Write(address, (int8_t*)&data, size);
				break;
			}
			default:{
				dumpregs();
				stringstream ss;
				ss << GetName() << ": unable to write to unmapped memory space 0x" << std::hex << address << ".";
				throw std::runtime_error(ss.str());
			}
		}
		
		return; //succefully written		
	}
		
	//may the request memory space be out of the mapped memory range, we assume
	//the code is pointing to some of the special addresses		
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
		case IRQ_CAUSE:	s->cause = value; return;
		case IRQ_MASK:		s->mask = value; return;
		case IRQ_EPC:		s->epc = value; return;
		case COUNTER:		s->counter = value; return;
		case COMPARE:		s->compare = value; s->cause &= 0xffef; return;
		case COMPARE2:		s->compare2 = value; s->cause &= 0xffdf; return;

		case DEBUG_ADDR: output_debug << (int8_t)(value & 0xff) << std::flush; return;
		case UART_WRITE: output_uart << (int8_t)(value & 0xff) << std::flush; return;
		case UART_DIVISOR: return;

		case EXIT_TRAP:
			std::cout << GetName() << ": exit trap triggered! " << std::endl;
			dumpregs();
			output_debug.close();
			output_uart.close();
            abort();
			return;
	}
		
	//if none of the special address has been reach, the requested
	//address if unknown to the system and we should halt the simulation
	dumpregs();
	stringstream ss;
			
	ss << GetName() << ": unable to write to unmapped memory space 0x" << std::hex << address << ".";
	throw std::runtime_error(ss.str());
}

#ifdef HFRISCV_ENABLE_COUNTERS

//Counters' getters
USignal<uint32_t>* THFRiscV::GetSignalCounterArith(){
	return _counter_iarith;
}
USignal<uint32_t>* THFRiscV::GetSignalCounterLogical(){
	return _counter_ilogical;
}
USignal<uint32_t>* THFRiscV::GetSignalCounterShift(){
	return _counter_ishift;
}
USignal<uint32_t>* THFRiscV::GetSignalCounterBranches(){
	return _counter_ibranches;
}
USignal<uint32_t>* THFRiscV::GetSignalCounterJumps(){
	return _counter_ijumps;
}
USignal<uint32_t>* THFRiscV::GetSignalCounterLoadStore(){
	return _counter_iloadstore;
}

//cycles
USignal<uint32_t>* THFRiscV::GetSignalCounterCyclesTotal(){
	return _counter_cycles_total;
}
USignal<uint32_t>* THFRiscV::GetSignalCounterCyclesStall(){
	return _counter_cycles_stall;
}
USignal<uint32_t>* THFRiscV::GetSignalHostTime(){
	return _counter_hosttime;
}

/**
 * Initialize Counters
 * memory-mapped address of counters must be informed
 */

void THFRiscV::InitCounters(
		uint32_t arith_counter_addr, 
		uint32_t logical_counter_addr,
		uint32_t shift_counter_addr,
		uint32_t branches_counter_addr,
		uint32_t jumps_counter_addr, 
		uint32_t loadstore_counter_addr,
		uint32_t cycles_total_counter_addr, 
		uint32_t cycles_stall_counter_addr,
		uint32_t hosttime_addr){

	_counter_iarith     = new USignal<uint32_t>(arith_counter_addr, GetName() + ".counters.iarith");
	_counter_ilogical   = new USignal<uint32_t>(logical_counter_addr, GetName() + ".counters.ilogical");
	_counter_ishift     = new USignal<uint32_t>(shift_counter_addr, GetName() + ".counters.ishift");
	_counter_ibranches  = new USignal<uint32_t>(branches_counter_addr, GetName() + ".counters.ibranches");
	_counter_ijumps     = new USignal<uint32_t>(jumps_counter_addr, GetName() + ".counters.ijumps");
	_counter_iloadstore = new USignal<uint32_t>(loadstore_counter_addr, GetName() + ".counters.iloadstore");

	_counter_cycles_total = new USignal<uint32_t>(cycles_total_counter_addr, GetName() + ".counters.cycles_total");
	_counter_cycles_stall = new USignal<uint32_t>(cycles_stall_counter_addr, GetName() + ".counters.cycles_stall");
	
	_counter_hosttime = new USignal<uint32_t>(hosttime_addr, GetName() + ".counters.hosttime");
}

/**
 * Update Counters
 * Counters are incremented by one every time some instruction
 * is executed. Please note that NOP and MOVE instructions are
 * ignored as riscv32i does not generate them.
*/
void THFRiscV::UpdateCounters(int opcode, int funct3){

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

		case 0x13: //type R
		case 0x33:
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
		default:
			std::cout << "wrn: opcode not accounted for energy estimation" << std::endl;
			break;
	}
	
}
#endif /* HFRISCV_ENABLE_COUNTERS */

SimulationTime THFRiscV::Run(){

	//call ancestor method which handles 
	//generic tasks all processor models 
	TProcessorBase::Run();

	//return 1; //TODO: REMOVETHSI!!!!!

	//update "external counters"
	s->counter++;

	if ((s->compare2 & 0xffffff) == (s->counter & 0xffffff)) s->cause |= 0x20;      /*IRQ_COMPARE2*/
	if (s->compare == s->counter) s->cause |= 0x10;                                 /*IRQ_COMPARE*/
	
	if (!(s->counter & 0x10000)) s->cause |= 0x8; else s->cause &= 0xfffffff7;      /*IRQ_COUNTER2_NOT*/
	if (s->counter & 0x10000) s->cause |= 0x4; else s->cause &= 0xfffffffb;         /*IRQ_COUNTER2*/
	if (!(s->counter & 0x40000)) s->cause |= 0x2; else s->cause &= 0xfffffffd;      /*IRQ_COUNTER_NOT*/
	if (s->counter & 0x40000) s->cause |= 0x1; else s->cause &= 0xfffffffe;         /*IRQ_COUNTER*/
	
	if (_signal_intr->Read() == 0x1) s->cause |= 0x100; else s->cause &= 0xfffffeff;/*IRQ_NOC*/

	//skip current cycle if stall is risen
	if(_signal_stall->Read() == 0x1){
		#ifdef HFRISCV_ENABLE_COUNTERS
		_counter_cycles_stall->Inc(1);
		#endif
		return 1;
	}

	#ifdef HFRISCV_ENABLE_COUNTERS
	_counter_cycles_total->Inc(1);
	#endif

	#ifdef HFRISCV_CYCLE_ACCURACY
	uint32_t pc_next_prediction;
	#endif
		
	uint32_t inst, i;
	uint32_t opcode, rd, rs1, rs2, funct3, funct7, imm_i, imm_s, imm_sb, imm_u, imm_uj;
	int32_t *r = s->r;
	uint32_t *u = (uint32_t *)s->r;
	uint32_t ptr_l, ptr_s;
	
	#ifdef HFRISCV_CYCLE_ACCURACY
	uint32_t branch_taken = 0;
	#endif
	
	if (s->status && (s->cause & s->mask)){
		s->epc = PC_NEXT;
		PC = s->vector;
		PC_NEXT = s->vector + 4;
		s->status = 0;
		for (i = 0; i < 4; i++)
			s->status_dly[i] = 0;
	}

	//FETCH STAGE
	GetMemory()->Read(PC, (int8_t*)&inst, 4); //4 x sizeof(uint8_t)

	//DECODE
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
		case 0x17: r[rd] = PC + imm_u; break;									/* AUIPC */
		
		case 0x6f: r[rd] = PC_NEXT; PC_NEXT = PC + imm_uj; break;				  /* JAL */
		case 0x67: r[rd] = PC_NEXT; PC_NEXT = (r[rs1] + imm_i) & 0xfffffffe; break; /* JALR */
		case 0x63:
			/* Branch prediction may fail if jumping 0 positions.
			TODO: check whether the architecture predict such jumps */
			
			#ifdef HFRISCV_CYCLE_ACCURACY
			pc_next_prediction = PC_NEXT;
			#endif
			
			switch(funct3){
				case 0x0: if (r[rs1] == r[rs2]){ PC_NEXT = PC + imm_sb; } break;	/* BEQ */
				case 0x1: if (r[rs1] != r[rs2]){ PC_NEXT = PC + imm_sb; } break;	/* BNE */
				case 0x4: if (r[rs1] < r[rs2]){ PC_NEXT = PC + imm_sb; } break;	/* BLT */
				case 0x5: if (r[rs1] >= r[rs2]){ PC_NEXT = PC + imm_sb; } break;	/* BGE */
				case 0x6: if (u[rs1] < u[rs2]){ PC_NEXT = PC + imm_sb; } break;	/* BLTU */
				case 0x7: if (u[rs1] >= u[rs2]){ PC_NEXT = PC + imm_sb; } break;	/* BGEU */
				default: goto fail;
			}
			
			#ifdef HFRISCV_CYCLE_ACCURACY
			branch_taken = (pc_next_prediction != PC_NEXT);
			#endif 
			
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
			if (funct7 == 0x1){											/* RV32M */
				switch(funct3){
					case 0:	r[rd] = (((int64_t)r[rs1] * (int64_t)r[rs2]) & 0xffffffff); break;		    /* MUL */
					case 1:	r[rd] = ((((int64_t)r[rs1] * (int64_t)r[rs2]) >> 32) & 0xffffffff); break;	/* MULH */
					case 2:	r[rd] = ((((int64_t)r[rs1] * (uint64_t)u[rs2]) >> 32) & 0xffffffff); break;	/* MULHSU */
					case 3:	r[rd] = ((((uint64_t)u[rs1] * (uint64_t)u[rs2]) >> 32) & 0xffffffff); break;/* MULHU */
					case 4:	if (r[rs2]) r[rd] = r[rs1] / r[rs2]; else r[rd] = 0; break;					/* DIV */
					case 5:	if (r[rs2]) r[rd] = u[rs1] / u[rs2]; else r[rd] = 0; break;					/* DIVU */
					case 6:	if (r[rs2]) r[rd] = r[rs1] % r[rs2]; else r[rd] = 0; break;					/* REM */
					case 7:	if (r[rs2]) r[rd] = u[rs1] % u[rs2]; else r[rd] = 0; break;					/* REMU */
					default: goto fail;
				}
				break;
			}else{
				switch(funct3){
					case 0x0:
						switch(funct7){
							case 0x0: r[rd] = r[rs1] + r[rs2]; break;				/* ADD */
							case 0x20: r[rd] = r[rs1] - r[rs2]; break;				/* SUB */
							default: goto fail;
						}
						break;
					case 0x1: r[rd] = r[rs1] << r[rs2]; break;						/* SLL */
					case 0x2: r[rd] = r[rs1] < r[rs2]; break;		 				/* SLT */
					case 0x3: r[rd] = u[rs1] < u[rs2]; break;		 				/* SLTU */
					case 0x4: r[rd] = r[rs1] ^ r[rs2]; break;						/* XOR */
					case 0x5:
						switch(funct7){
							case 0x0: r[rd] = u[rs1] >> u[rs2]; break;				/* SRL */
							case 0x20: r[rd] = r[rs1] >> r[rs2]; break;				/* SRA */
							default: goto fail;
						}
						break;
					case 0x6: r[rd] = r[rs1] | r[rs2]; break;						/* OR */
					case 0x7: r[rd] = r[rs1] & r[rs2]; break;						/* AND */
					default: goto fail;
				}
				break;
			}
			break;
		default:
fail:
			stringstream ss;
			ss << GetName() << ":invalid opcode (pc=0x" << std::hex << PC;
			ss << " opcode=0x" << std::hex << inst << ")";
	
			dumpregs();
			bp(s, RISCV_INVALID_OPCODE);
			
			throw std::runtime_error(ss.str());
			break;
	}
	
	_last_pc = PC;
	PC = PC_NEXT;
	PC_NEXT = PC_NEXT + 4;
	s->status = s->status_dly[0];
	
	for (i = 0; i < 3; i++)
		s->status_dly[i] = s->status_dly[i+1];

	//MOVI DAQUI
	
	//s->counter++;
			
	//if ((s->compare2 & 0xffffff) == (s->counter & 0xffffff)) s->cause |= 0x20;      /*IRQ_COMPARE2*/
	//if (s->compare == s->counter) s->cause |= 0x10;                                 /*IRQ_COMPARE*/
	
	//if (!(s->counter & 0x10000)) s->cause |= 0x8; else s->cause &= 0xfffffff7;      /*IRQ_COUNTER2_NOT*/
	//if (s->counter & 0x10000) s->cause |= 0x4; else s->cause &= 0xfffffffb;         /*IRQ_COUNTER2*/
	//if (!(s->counter & 0x40000)) s->cause |= 0x2; else s->cause &= 0xfffffffd;      /*IRQ_COUNTER_NOT*/
	//if (s->counter & 0x40000) s->cause |= 0x1; else s->cause &= 0xfffffffe;         /*IRQ_COUNTER*/
	
	//if (_signal_intr->Read() == 0x1) s->cause |= 0x100; else s->cause &= 0xfffffeff;/*IRQ_NOC*/
	

	#ifdef HFRISCV_ENABLE_COUNTERS
	UpdateCounters(opcode, funct3);
	#endif
	
	//When in cycle-accuracy mode, takes three cycles per instruction, 
	//except for those of memory I/O. In the later case. Since we simulate 
	//the pipeline by executing one instruction per cycle (starting from 
	//the 3th cycle), adding 1 cycle to simulate I/O delay. We also calculate
	//branch prediction.

	#ifdef HFRISCV_CYCLE_ACCURACY
	switch(opcode){
		case 0x63:
			if(branch_taken){
				return 1;
			}else{
				return 2;
			}
			break;
		case 0x23:
		case 0x3:
			return 2;
			break;
		default:
			return 1;
			break;
	}
	#else

	//When in instruction mode, evey instruction takes exactly one cycle to
	//leave the pipeline. This mode aims for performance.
	return 1;
	#endif
}

THFRiscV::THFRiscV(std::string name, USignal<uint8_t>* intr, USignal<uint8_t>* stall, UMemory* mainmem)
	: TProcessorBase(name, HFRISCV_PC_MEMBASE, mainmem) {

	s = new risc_v_state;
	memset(s, 0, sizeof(risc_v_state));
	
	PC = HFRISCV_PC_MEMBASE;
	PC_NEXT = PC + 4;
	
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
	//s->cycles = 0;
	
	//set interruption wire (to be managed by the top-level module)
	_signal_intr = intr;
	_signal_stall = stall;

	output_debug.open("logs/" + GetName() + "_debug.log", std::ofstream::out | std::ofstream::trunc);
	output_uart.open("logs/" + GetName() + "_uart.log", std::ofstream::out | std::ofstream::trunc);
}

//TODO: clear allocated memory if any
THFRiscV::~THFRiscV(){
	
	#ifdef HFRISCV_ENABLE_COUNTERS
	delete _counter_iarith;
	delete _counter_ilogical;
	delete _counter_ishift;
	delete _counter_ibranches;
	delete _counter_ijumps;
	delete _counter_iloadstore;
	
	delete _counter_cycles_total;
	delete _counter_cycles_stall;
	
	delete _counter_hosttime;

	#endif
}

void THFRiscV::Reset(){
    //TODO: to be implemented
    return;
}

//api access shorthands (must be undef at the end of the file)
#undef PC 
#undef PC_NEXT 
#undef R
