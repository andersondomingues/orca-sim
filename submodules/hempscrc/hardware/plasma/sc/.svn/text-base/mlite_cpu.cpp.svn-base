#include "mlite_cpu.h"

/***  Copoprocessor 0 registers map
*		- $10: page
*		- $12: intr_enable
*		- $14: state->epc
***/

/*** OBS: 'mem_pause' input stalls the CPU ONLY when executing memory acess (Load/Store) ***/

#ifdef MTI_SYSTEMC
SC_MODULE_EXPORT(mlite_cpu);
#endif

/*** Process thread ***/
void mlite_cpu::mlite() {

	for(;;) {

		current_page.write(page>>shift);

		if ( !mem_pause )
			pc_last = state->pc;	// Stores the last pc address

		if ( reset_in.read() ) {
			page = 0;
			intr_enable = false;
			prefetch = false;
			jump_or_branch = false;
			no_execute_branch_delay_slot = false;
			state->pc = 0;
			mem_byte_we.write(0x0);
			mem_address.write(state->pc);
			word_addr = -4;					// 0xFFFFFFFC
			wait(17);
		}
		else {
			r = (int*)state->r;			// Signed mask.
			u = (unsigned int*)state->r;	// Unsigned mask.


			if ( intr_in.read() && intr_enable && !jump_or_branch ) {	// Does not interrupt a Branch Delay Slot.
				state->epc = state->pc - 4;
				state->pc = 0x3C;
				page = 0;
				mem_address.write(state->pc);
				opcode = mem_data_r.read();
				wait(1);
				intr_enable = false;
				no_execute_branch_delay_slot = true;	// Does not execute the Branch Delay Slot instruction relative to the ISR jump.
				continue;		// Executes the loaded instruction after the ISR.
			}
			else
				state->pc += 4;


			// Adds the page number.
			state->pc |= page;

			// Instruction read.
			mem_address.write(state->pc);
			if ( prefetch ) {
				opcode = prefetched_opcode;
				prefetch = false;
			}
			else
				opcode = mem_data_r.read();

			op = (opcode >> 26) & 0x3f;
			rs = (opcode >> 21) & 0x1f;
			rt = (opcode >> 16) & 0x1f;
			rd = (opcode >> 11) & 0x1f;
			re = (opcode >> 6) & 0x1f;
			func = opcode & 0x3f;
			imm = opcode & 0xffff;
			imm_shift = (((int)(short)imm) << 2) - 4;
			target = (opcode << 6) >> 4;
			ptr = (short)imm + r[rs];
			ptr |= page;	// Adds the page number.


			r[0] = 0;		// Constant.


			jump_or_branch = false;


			if (opcode == 0 || no_execute_branch_delay_slot ) { /*NOP*/
				wait(1);
				no_execute_branch_delay_slot = false;
				continue;
			}


			// Instruction decode, execution and write back.
			switch(op) {
				case 0x00:/*SPECIAL*/
					switch(func) {
						case 0x00:/*SLL*/
							wait(1);
							r[rd]=r[rt]<<re;
						break;

						case 0x02:/*SRL*/
							wait(1);
							r[rd]=u[rt]>>re;
						break;

						case 0x03:/*SRA*/
							wait(1);
							r[rd]=r[rt]>>re;
						break;

						case 0x04:/*SLLV*/
							wait(1);
							r[rd]=r[rt]<<r[rs];
						break;

						case 0x06:/*SRLV*/
							wait(1);
							r[rd]=u[rt]>>r[rs];
						break;

						case 0x07:/*SRAV*/
							wait(1);
							r[rd]=r[rt]>>r[rs];
						break;

						case 0x08:/*JR*/
							jump_or_branch = true;
							state->pc = r[rs];
							state->pc |= page;
							mem_address.write(state->pc);
							wait(1);
						break;

						case 0x09:/*JALR*/
							jump_or_branch = true;
							r[rd] = state->pc;
							state->pc = r[rs];
							state->pc |= page;
							mem_address.write(state->pc);
							wait(1);
						break;

						case 0x0a:/*MOVZ*/
							wait(1);
							if ( !r[rt] )
								r[rd] = r[rs];
						break;  /*IV*/

						case 0x0b:/*MOVN*/
							wait(1);
							if ( r[rt] )
								r[rd] = r[rs];
						break;  /*IV*/

						case 0x0c:/*SYSCALL*/
							state->epc = state->pc;
							state->pc = 0x44;
							page = 0;
							current_page.write(page>>shift);
							mem_address.write(state->pc);
							wait(1);
							intr_enable = false;
						break;

						case 0x0d:/*BREAK*/
							wait(1);
						break;

						case 0x0f:/*SYNC*/
							wait(1);
						break;

						case 0x10:/*MFHI*/
							wait(1);
							r[rd] = state->hi;
						break;

						case 0x11:/*MTHI*/
							wait(1);
							state->hi = r[rs];
						break;

						case 0x12:/*MFLO*/
							wait(1);
							r[rd] = state->lo;
						break;

						case 0x13:/*MTLO*/
							wait(1);
							state->lo = r[rs];
						break;

						case 0x18:/*MULT*/
							wait(1);

							prefetch = true;
							prefetched_opcode = mem_data_r.read();
							mem_address.write(state->pc);
							wait(31);

							mult_big_signed(r[rs],r[rt]);
							//sc_int<64> result;
							//result = r[rs] * r[rt];
							//state->hi = result.range(63,32);
							//state->lo = result.range(31,0);
						break;

						case 0x19:/*MULTU*/
							//state->lo=r[rs]*r[rt]; state->hi=0; break;
							wait(1);

							prefetch = true;
							prefetched_opcode = mem_data_r.read();
							mem_address.write(state->pc);
							wait(31);

							mult_big(r[rs],r[rt]);
							//sc_uint<64> result;
							//result = u[rs] * u[rt];
							//state->hi = result.range(63,32);
							//state->lo = result.range(31,0);
						break;

						case 0x1a:/*DIV*/
							wait(1);

							prefetch = true;
							prefetched_opcode = mem_data_r.read();
							mem_address.write(state->pc);
							wait(31);
							
							if(r[rt] == 0){
							printf("%d\t %d\n ",state->pc,opcode);
							}
							state->lo = (r[rt]>0) ? r[rs] / r[rt] : 0;
							state->hi = (r[rt]>0) ? r[rs] % r[rt] : r[rs];
					//		state->lo = r[rs] / r[rt];						
					//		state->hi = r[rs] % r[rt];
							
						break;

						case 0x1b:/*DIVU*/
							wait(1);

							prefetch = true;
							prefetched_opcode = mem_data_r.read();
							mem_address.write(state->pc);
							wait(31);

							state->lo = u[rs] / u[rt];
							state->hi = u[rs] % u[rt];
						break;

						case 0x20:/*ADD*/
							wait(1);
							r[rd] = r[rs] + r[rt];
						break;

						case 0x21:/*ADDU*/
							wait(1);
							r[rd] = r[rs] + r[rt];
						break;

						case 0x22:/*SUB*/
							wait(1);
							r[rd] = r[rs] - r[rt];
						break;

						case 0x23:/*SUBU*/
							wait(1);
							r[rd] = r[rs] - r[rt];
						break;

						case 0x24:/*AND*/
							wait(1);
							r[rd] = r[rs] & r[rt];
						break;

						case 0x25:/*OR*/
							wait(1);
							r[rd] = r[rs] | r[rt];
						break;

						case 0x26:/*XOR*/
							wait(1);
							r[rd] = r[rs] ^ r[rt];
						break;

						case 0x27:/*NOR*/
							wait(1);
							r[rd] = ~(r[rs] | r[rt]);
						break;

						case 0x2a:/*SLT*/
							wait(1);
							r[rd]= (r[rs] < r[rt]);
						break;

						case 0x2b:/*SLTU*/
							wait(1);
							r[rd] = (u[rs] < u[rt]);
						break;

						case 0x2d:/*DADDU*/
							wait(1);
							r[rd] = r[rs] + u[rt];
						break;

						case 0x31:/*TGEU*/ wait(1); break;
						case 0x32:/*TLT*/  wait(1); break;
						case 0x33:/*TLTU*/ wait(1); break;
						case 0x34:/*TEQ*/  wait(1); break;
						case 0x36:/*TNE*/  wait(1); break;
						default:
							printf("\nERROR0 address=0x%x opcode=0x%x\n",state->pc,opcode);
							return;
					}
				break;

				case 0x01:/*REGIMM*/
					switch(rt) {
						case 0x10:/*BLTZAL*/
							if ( r[rs] < 0 ) {
								jump_or_branch = true;
								r[31] = state->pc;
								state->pc += imm_shift;
								mem_address.write(state->pc);
							}
							wait(1);
						break;

						case 0x00:/*BLTZ*/
							if ( r[rs] < 0 ) {
								jump_or_branch = true;
								state->pc += imm_shift;
								mem_address.write(state->pc);
							}
							wait(1);
						break;

						case 0x11:/*BGEZAL*/
							if ( r[rs] >= 0 ) {
								jump_or_branch = true;
								r[31] = state->pc;
								state->pc += imm_shift;
								mem_address.write(state->pc);
							}
							wait(1);
						break;

						case 0x01:/*BGEZ*/
							if ( r[rs] >= 0 ) {
								jump_or_branch = true;
								state->pc += imm_shift;
								mem_address.write(state->pc);
							}
							wait(1);
						break;

						case 0x12:/*BLTZALL*/
							if ( r[rs] < 0 ) {
								jump_or_branch = true;
								r[31] = state->pc;
								state->pc += imm_shift;
								mem_address.write(state->pc);
							}
							wait(1);
						break;

						case 0x02:/*BLTZL*/
							if ( r[rs] < 0 ) {
								jump_or_branch = true;
								state->pc += imm_shift;
								mem_address.write(state->pc);
							}
							wait(1);
						break;

						case 0x13:/*BGEZALL*/
							if ( r[rs] >= 0 ) {
								jump_or_branch = true;
								r[31] = state->pc;
								state->pc += imm_shift;
								mem_address.write(state->pc);
							}
							wait(1);
						break;

						case 0x03:/*BGEZL*/
							if ( r[rs] >= 0 ) {
								jump_or_branch = true;
								state->pc += imm_shift;
								mem_address.write(state->pc);
							}
							wait(1);
						break;

						default:
							printf("\nERROR1 address=0x%x opcode=0x%x\n",state->pc,opcode);
							return;
					}
				break;

				case 0x03:/*JAL*/
					jump_or_branch = true;
					r[31] = state->pc;
					state->pc = (state->pc & 0xf0000000) | target;
					state->pc |= page;				// Adds the page number.
					mem_address.write(state->pc);
					wait(1);
				break;

				case 0x02:/*J*/
					jump_or_branch = true;
					state->pc = (state->pc & 0xf0000000) | target;
					state->pc |= page;				// Adds the page number.
					mem_address.write(state->pc);
					wait(1);
				break;

				case 0x04:/*BEQ*/
					if ( r[rs] == r[rt] ) {
						jump_or_branch = true;
						state->pc += imm_shift;
						mem_address.write(state->pc);
					}
					wait(1);
				break;

				case 0x05:/*BNE*/
					if ( r[rs] != r[rt] ) {
						jump_or_branch = true;
						state->pc += imm_shift;
						mem_address.write(state->pc);
					}
					wait(1);
				break;

				case 0x06:/*BLEZ*/
					if ( r[rs] <= 0 ) {
						jump_or_branch = true;
						state->pc += imm_shift;
						mem_address.write(state->pc);
					}
					wait(1);
				break;

				case 0x07:/*BGTZ*/
					if ( r[rs] > 0 ) {
						jump_or_branch = true;
						state->pc += imm_shift;
						mem_address.write(state->pc);
					}
					wait(1);
				break;

				case 0x08:/*ADDI*/
					wait(1);
					r[rt] = r[rs] + (short)imm;
				break;

				case 0x09:/*ADDIU*/
					wait(1);
					u[rt] = u[rs] + (short)imm;
				break;

				case 0x0a:/*SLTI*/
					wait(1);
					r[rt] = r[rs] < (short)imm;
				break;

				case 0x0b:/*SLTIU*/
					wait(1);
					u[rt] = u[rs] < (unsigned int)(short)imm;
				break;

				case 0x0c:/*ANDI*/
					wait(1);
					r[rt] = r[rs] & imm;
				break;

				case 0x0d:/*ORI*/
					wait(1);
					r[rt] = r[rs] | imm;
				break;

				case 0x0e:/*XORI*/
					wait(1);
					r[rt] = r[rs] ^ imm;
				break;

				case 0x0f:/*LUI*/
					wait(1);
					r[rt] = (imm<<16);
				break;

				case 0x10:/*COP0*/
					wait(1);

					if ( opcode & (1<<23) )	{/*MTC0*/
						switch (rd) {
							case 10:
								page = r[rt];
							break;

							case 12:
								intr_enable = r[rt] ;
							break;

							case 14:
								state->epc = r[rt];
							break;

							default:
								printf("MTC0: reg %d not mapped.\n",rd);
								return;
						}
					}
					else { /*MFC0*/
						switch (rd) {
							case 10:
								r[rt] = page;
							break;

							case 12:
								r[rt] = intr_enable;
							break;

							case 14:
								r[rt] = state->epc;
							break;

							default:
								printf("MFC0: reg %d not mapped.\n",rd);
								return;
						}
					}
				break;

		//      case 0x11:/*COP1*/ break;
		//      case 0x12:/*COP2*/ break;
		//      case 0x13:/*COP3*/ break;

				case 0x14:/*BEQL*/
					if ( r[rs] == r[rt] ) {
						jump_or_branch = true;
						state->pc += imm_shift;
						mem_address.write(state->pc);
					}
					wait(1);
				break;

				case 0x15:/*BNEL*/
					if ( r[rs] != r[rt] ) {
						jump_or_branch = true;
						state->pc += imm_shift;
						mem_address.write(state->pc);
					}
					wait(1);
				break;

				case 0x16:/*BLEZL*/
					if ( r[rs] <= 0 ) {
						jump_or_branch = true;
						state->pc += imm_shift;
						mem_address.write(state->pc);
					}
					wait(1);
				break;

				case 0x17:/*BGTZL*/
					if ( r[rs] > 0 ) {
						jump_or_branch = true;
						state->pc += imm_shift;
						mem_address.write(state->pc);
					}
					wait(1);
				break;

		//      case 0x1c:/*MAD*/  break;   /*IV*/

				case 0x20:/*LB*/
					mem_address.write(ptr & word_addr);	// Address the memory with word address.
					wait(1);

					// Verifies the mem_pause signal at the first execution cycle
					if ( mem_pause.read() ) {
						mem_address.write(pc_last);	// Keep the last memory address before mem_pause = '1'

						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(ptr & word_addr);// Address the memory with word address.
						wait(1);
					}

					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					// Verifies the mem_pause signal at the second execution cycle
					if (mem_pause.read()) {
						mem_address.write(ptr & word_addr);// Keep the last memory address before mem_pause = '1'

						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(state->pc);// Address the next instruction
						wait(1);
					}

					if ((ptr & 3) == 3)
						if (big_endian)
							r[rt] = (char)mem_data_r.read().range(7,0);
						else
							r[rt] = (char)mem_data_r.read().range(31,24);

					else if ((ptr & 2) == 2)
						if (big_endian)
							r[rt] = (char)mem_data_r.read().range(15,8);
						else
							r[rt] = (char)mem_data_r.read().range(23,16);

					else if ((ptr & 1) == 1)
						if (big_endian)
							r[rt] = (char)mem_data_r.read().range(23,16);
						else
							r[rt] = (char)mem_data_r.read().range(15,8);

					else
						if (big_endian)
							r[rt] = (char)mem_data_r.read().range(31,24);
						else
							r[rt] = (char)mem_data_r.read().range(7,0);


				break;

				case 0x21:/*LH*/
					//assert((ptr & 1) == 0);
					mem_address.write(ptr & word_addr);	// Address the memory with word address.
					wait(1);

					// Verifies the mem_pause signal at the first execution cycle
					if ( mem_pause.read() ) {
						mem_address.write(pc_last);	// Keep the last memory address before mem_pause = '1'

						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(ptr & word_addr);// Address the memory with word address.
						wait(1);
					}

					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					// Verifies the mem_pause signal at the second execution cycle
					if (mem_pause.read()) {
						mem_address.write(ptr & word_addr);// Keep the last memory address before mem_pause = '1'

						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(state->pc);// Address the next instruction
						wait(1);
					}

					if ((ptr & 2) == 2)
						if (big_endian)
							r[rt] = (short)mem_data_r.read().range(15,0);
						else
							r[rt] = (short)mem_data_r.read().range(31,16);
					else
						if (big_endian)
							r[rt] = (short)mem_data_r.read().range(31,16);
						else
							r[rt] = (short)mem_data_r.read().range(15,0);
				break;

				case 0x22:/*LWL*/  rt=rt; //fixme fall through
				case 0x23:/*LW*/
					//assert((ptr & 3) == 0);
					mem_address.write(ptr & word_addr);	// Address the memory with word address.
					wait(1);

					// Verifies the mem_pause signal at the first execution cycle
					if ( mem_pause.read() ) {
						mem_address.write(pc_last);	// Keep the last memory address before mem_pause = '1'
						wait(1);
						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(ptr & word_addr);// Address the memory with word address.
						wait(1);
					}

					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					// Verifies the mem_pause signal at the second execution cycle
					if (mem_pause.read()) {
						mem_address.write(ptr & word_addr);// Keep the last memory address before mem_pause = '1'

						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(state->pc);// Address the next instruction
						wait(1);
					}

					r[rt] = mem_data_r.read();
				break;

				case 0x24:/*LBU*/
					mem_address.write(ptr & word_addr);	// Address the memory with word address.
					wait(1);

					// Verifies the mem_pause signal at the first execution cycle
					if ( mem_pause.read() ) {
						mem_address.write(pc_last);	// Keep the last memory address before mem_pause = '1'

						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(ptr & word_addr);// Address the memory with word address.
						wait(1);
					}

					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					// Verifies the mem_pause signal at the second execution cycle
					if (mem_pause.read()) {
						mem_address.write(ptr & word_addr);// Keep the last memory address before mem_pause = '1'

						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(state->pc);// Address the next instruction
						wait(1);
					}

					if ((ptr & 3) == 3)
						if (big_endian)
								r[rt] = (unsigned char)mem_data_r.read().range(7,0);
							else
								r[rt] = (unsigned char)mem_data_r.read().range(31,24);

						else if ((ptr & 2) == 2)
							if (big_endian)
								r[rt] = (unsigned char)mem_data_r.read().range(15,8);
							else
								r[rt] = (unsigned char)mem_data_r.read().range(23,16);

						else if ((ptr & 1) == 1)
							if (big_endian)
								r[rt] = (unsigned char)mem_data_r.read().range(23,16);
							else
								r[rt] = (unsigned char)mem_data_r.read().range(15,8);

						else
							if (big_endian)
								r[rt] = (unsigned char)mem_data_r.read().range(31,24);
							else
								r[rt] = (unsigned char)mem_data_r.read().range(7,0);
				break;

				case 0x25:/*LHU*/
					//assert((ptr & 1) == 0);
					mem_address.write(ptr & word_addr);	// Address the memory with word address.
					wait(1);

					// Verifies the mem_pause signal at the first execution cycle
					if ( mem_pause.read() ) {
						mem_address.write(pc_last);	// Keep the last memory address before mem_pause = '1'
						wait(1);
						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(ptr & word_addr);// Address the memory with word address.
						wait(1);
					}

					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					// Verifies the mem_pause signal at the second execution cycle
					if (mem_pause.read()) {
						mem_address.write(ptr & word_addr);// Keep the last memory address before mem_pause = '1'

						while (mem_pause.read())	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(state->pc);// Address the next instruction
						wait(1);
					}

					if ((ptr & 2) == 2)
						if (big_endian)
							r[rt] = (unsigned short)mem_data_r.read().range(15,0);
						else
							r[rt] = (unsigned short)mem_data_r.read().range(31,16);
					else
						if (big_endian)
							r[rt] = (unsigned short)mem_data_r.read().range(31,16);
						else
							r[rt] = ( unsigned short)mem_data_r.read().range(15,0);
				break;

				case 0x26:/*LWR*/  wait(1); break; //fixme

				case 0x28:/*SB*/
					byte_write = r[rt] & 0x000000FF;	/* Retrieves the byte to be stored */
					mem_address.write(ptr & word_addr);	// Address the memory with word address.
					mem_data_w.write((byte_write<<24) | (byte_write<<16) | (byte_write<<8) | byte_write);

					if ((ptr & 3) == 3)
						if (big_endian)
							byte_en = 0x1;
						else
							byte_en = 0x8;

					else if ((ptr & 2) == 2)
						if (big_endian)
							byte_en = 0x2;
						else
							byte_en = 0x4;

					else if ((ptr & 1) == 1)
						if (big_endian)
							byte_en = 0x4;
						else
							byte_en = 0x2;

					else
						if (big_endian)
							byte_en = 0x8;
						else
							byte_en = 0x1;

					mem_byte_we.write(byte_en);

					wait(1);

					// Verifies the mem_pause signal at the first execution cycle
					if ( mem_pause ) {
						mem_address.write(pc_last);// Keep the last memory address before mem_pause = '1'
						mem_byte_we.write(0);// Disable write

						while (mem_pause)	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(ptr & word_addr);	// Address the memory with word address.
						mem_byte_we.write(byte_en);			// Enable write
						wait(1);
					}

					mem_byte_we.write(0x0);
					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					// Verifies the mem_pause signal at the second execution cycle
					if (mem_pause) {
						mem_address.write(ptr & word_addr);// Keep the last memory address before mem_pause = '1'

						while (mem_pause)	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(state->pc);// Address the next instruction
						wait(1);
					}
				break;

				case 0x29:/*SH*/
					byte_write = r[rt] & 0x0000FFFF;	/* Retrieves the half word to be stored */
					mem_address.write(ptr & word_addr);	// Address the memory with word address.
					mem_data_w.write((byte_write<<16) | byte_write);

					if ((ptr & 2) == 2)
						if (big_endian)
							byte_en = 0x3;
						else
							byte_en = 0xC;
					else
						if (big_endian)
							byte_en = 0xC;
						else
							byte_en = 0x3;

					mem_byte_we.write(byte_en);

					wait(1);

					// Verifies the mem_pause signal at the first execution cycle
					if ( mem_pause ) {
						mem_address.write(pc_last);// Keep the last memory address before mem_pause = '1'
						mem_byte_we.write(0);// Disable write

						while (mem_pause)	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(ptr & word_addr);	// Address the memory with word address.
						mem_byte_we.write(byte_en);			// Enable write
						wait(1);
					}

					mem_byte_we.write(0x0);
					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					// Verifies the mem_pause signal at the second execution cycle
					if (mem_pause) {
						mem_address.write(ptr & word_addr);// Keep the last memory address before mem_pause = '1'

						while (mem_pause)	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(state->pc);// Address the next instruction
						wait(1);
					}
				break;

				case 0x2a:/*SWL*/  rt=rt; //fixme fall through
				case 0x2b:/*SW*/
					//assert((ptr & 3) == 0);
					mem_address.write(ptr);
					mem_data_w.write(r[rt]);
					mem_byte_we.write(0xF);
					wait(1);

					// Verifies the mem_pause signal at the first execution cycle
					if ( mem_pause ) {
						mem_address.write(pc_last);// Keep the last memory address before mem_pause = '1'
						mem_byte_we.write(0);// Disable write

						while (mem_pause)	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(ptr & word_addr);	// Address the memory with word address.
						mem_byte_we.write(0xF);			// Enable write
						wait(1);
					}

					mem_byte_we.write(0x0);
					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					// Verifies the mem_pause signal at the second execution cycle
					if (mem_pause) {
						mem_address.write(ptr & word_addr);// Keep the last memory address before mem_pause = '1'
						while (mem_pause)	// Stalls the CPU while mem_pause = '1'
							wait(1);

						mem_address.write(state->pc);// Address the next instruction
						wait(1);
					}
				break;

				case 0x2e:/*SWR*/  wait(1); break; //fixme
				case 0x2f:/*CACHE*/wait(1); break;

				case 0x30:/*LL*/
					//assert((ptr & 3) == 0);
					mem_address.write(ptr);
					wait(1);

					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					r[rt] = mem_data_r.read();
				break;

		//      case 0x31:/*LWC1*/ break;
		//      case 0x32:/*LWC2*/ break;
		//      case 0x33:/*LWC3*/ break;
		//      case 0x35:/*LDC1*/ break;
		//      case 0x36:/*LDC2*/ break;
		//      case 0x37:/*LDC3*/ break;
		//      case 0x38:/*SC*/     *(int*)ptr=r[rt]; r[rt]=1; break;

				case 0x38:/*SC*/
					//assert((ptr & 3) == 0);
					mem_address.write(ptr);
					mem_data_w.write(r[rt]);
					mem_byte_we.write(0xF);
					wait(1);

					mem_byte_we.write(0x0);
					prefetch = true;
					prefetched_opcode = mem_data_r.read();
					mem_address.write(state->pc);
					wait(1);

					r[rt] = 1;
				break;
		//
				case 0x39:/*SWC1*/ wait(1); break;
		//      case 0x3a:/*SWC2*/ break;
		//      case 0x3b:/*SWC3*/ break;
		//      case 0x3d:/*SDC1*/ break;
		//      case 0x3e:/*SDC2*/ break;
		//      case 0x3f:/*SDC3*/ break;
				default:
					printf("\nERROR2 address=0x%x opcode=0x%x\n",state->pc,opcode);
					return;
			}	// switch()
		}		// else
	}			// for(;;)
}


void mlite_cpu::mult_big(unsigned int a, unsigned int b) {

	sc_uint <64> ahi, alo, bhi, blo;
	sc_uint <64> c0, c1, c2;
	sc_uint <64> c1_a, c1_b;

	//printf("mult_big(0x%x, 0x%x)\n", a, b);
	ahi = a >> 16;
	alo = a & 0xffff;
	bhi = b >> 16;
	blo = b & 0xffff;

	c0 = alo * blo;
	c1_a = ahi * blo;
	c1_b = alo * bhi;
	c2 = ahi * bhi;

	c2 += (c1_a >> 16) + (c1_b >> 16);
	c1 = (c1_a & 0xffff) + (c1_b & 0xffff) + (c0 >> 16);
	c2 += (c1 >> 16);
	c0 = (c1 << 16) + (c0 & 0xffff);
	//printf("answer=0x%x 0x%x\n", c2, c0);
	state->hi = c2;
	state->lo = c0;
}

void mlite_cpu::mult_big_signed(int a, int b) {

	sc_uint <64> ahi, alo, bhi, blo;
	sc_uint <64> c0, c1, c2;
	sc_uint <64> c1_a, c1_b;

	//printf("mult_big_signed(0x%x, 0x%x)\n", a, b);
	ahi = a >> 16;
	alo = a & 0xffff;
	bhi = b >> 16;
	blo = b & 0xffff;

	c0 = alo * blo;
	c1_a = ahi * blo;
	c1_b = alo * bhi;
	c2 = ahi * bhi;

	c2 += (c1_a >> 16) + (c1_b >> 16);
	c1 = (c1_a & 0xffff) + (c1_b & 0xffff) + (c0 >> 16);
	c2 += (c1 >> 16);
	c0 = (c1 << 16) + (c0 & 0xffff);
	//printf("answer=0x%x 0x%x\n", c2, c0);
	state->hi = c2;
	state->lo = c0;
}

