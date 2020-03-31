/** 
 * This part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
 *
 * Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. **/

#ifndef __TMULT_H
#define __TMULT_H

//lib dependent includes
#include <iostream>
#include <vector>
#include <stdint.h>

//api includes
#include <TimedModel.h>
//#include <USignal.h>

// SIMD floating point sequential multiplier, capable of up to 16 mult in 'parallel'
#define VET_SEQ_MULT_RESULT		0xf0000240 // 420, 424, 428, ... 456
#define VET_SEQ_MULT_OP1		0xf0000340
#define VET_SEQ_MULT_OP2		0xf0000400

/**
 * This class models an external (memory-mapped) combinational FP multipler
 * TODO: does it work with signed values ?!?!
 */
class TimedFPMultiplier : public TimedModel{
	union DataMult {
	   uint32_t i;
	   float f;
	} ; 

	private:
		//#ifdef MULT_ENABLE_COUNTERS
		//	USignal<uint32_t>* _counter_active;
		//#endif

		union DataMult _op1;
		union DataMult _op2;

	public:	
		TimedFPMultiplier(std::string name): TimedModel(name) {
			_op1.i=0; 
			_op2.i=0;
		};
		~TimedFPMultiplier(){};

		// getters
		uint32_t GetResult();
		uint32_t GetOp1() {	return _op1.i; };
		uint32_t GetOp2() {	return _op2.i; };

		// setters
		void SetOp1(uint32_t op1) {
			//printf("setting op1/n");
			_op1.f = (float)op1;
		};

		void SetOp2(uint32_t op2) {	_op2.i = op2; };

		void Reset(){_op1.i=0; _op2.i=0; };
		
		/** Implementation of the Process' interface
		  * @return time taken for perming next cycle */
		SimulationTime Run();
};

#endif /* TMULT_H */

