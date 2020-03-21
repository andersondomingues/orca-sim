/** 
 * This file is part of project URSA. More information on the project
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
#include <iostream>
#include <UMultiplier.h>

UMultiplier::UMultiplier(std::string name) : UntimedModel(name){
    _name = name;
}

UMultiplier::~UMultiplier(){
	//nothing to do
}

void UMultiplier::SetOpA(uint32_t opa){
	_opa = opa;
}

void UMultiplier::SetOpB(uint32_t opb){
	_opb = opb;
}

uint32_t UMultiplier::GetResult(){
	return _opa * _opb;
}
