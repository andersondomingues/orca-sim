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
		
int32_t TArm7TDMI::fetch(){
	return 0;
}

int32_t TArm7TDMI::decode(){
	return 0;
}

int32_t TArm7TDMI::execute(){
	return 0;
}

void TArm7TDMI::Reset(){
	return;
}

TArm7TDMI::TArm7TDMI(string name) : TimedModel(name) {
}

TArm7TDMI::~TArm7TDMI(){

}

long long unsigned int TArm7TDMI::Run(){
	this->fetch();
	this->decode();
	this->execute();
	return 1;
}
