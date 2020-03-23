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
#ifndef __UMULT_H
#define __UMULT_H

//lib dependent includes
#include <iostream>
//#include <queue>
#include <vector>
#include <stdint.h>

//api includes
#include <UntimedModel.h>

// floating point multiplier
#define MULT_RESULT		    0xf0000100
#define MULT_OP1		    0xf0000104
#define MULT_OP2		    0xf0000108
// uint_32 multiplier
#define INT_MULT_RESULT		0xf000010C
#define INT_MULT_OP1		0xf0000110
#define INT_MULT_OP2		0xf0000114


/**
 * This class models an external (memory-mapped) combinational FP multipler
 * TODO: does it work with signed values ?!?!
 */
class UntimedFPMultiplier : public UntimedModel{

union DataMult {
   uint32_t i;
   float f;
} ; 

private:
	union DataMult _op1;
	union DataMult _op2;
public:	

	UntimedFPMultiplier(std::string name): UntimedModel(name) {_op1.i=0; _op2.i=0;};
	~UntimedFPMultiplier(){};

	// getters
	uint32_t GetResult();
	uint32_t GetOp1() {	return _op1.i;	};
	uint32_t GetOp2() {	return _op2.i;	};

	// setters
	void SetOp1(uint32_t op1) {	_op1.i = op1; 	};
	void SetOp2(uint32_t op2) {	_op2.i = op2; 	};

	void Reset(){_op1.i=0; _op2.i=0;};
};

/**
 * This class models an external (memory-mapped) combinational integer multipler
 * TODO: does it work with signed values ?!?!
 */
class UntimedIntMultiplier : public UntimedModel{

private:
	uint32_t _op1;
	uint32_t _op2;
public:	

	UntimedIntMultiplier(std::string name): UntimedModel(name) {_op1=0; _op2=0;};
	~UntimedIntMultiplier(){};

	// getters
	uint32_t GetResult();
	uint32_t GetOp1() {return _op1;};
	uint32_t GetOp2() {return _op2;};

	// setters
	void SetOp1(uint32_t op1) {_op1 = op1; };
	void SetOp2(uint32_t op2) {_op2 = op2; };

	void Reset(){_op1=0; _op2=0;};
};


#endif /* UMULT_H */
