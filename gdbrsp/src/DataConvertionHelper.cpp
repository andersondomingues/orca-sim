/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software
 * http://https://github.com/andersondomingues/orca-mpsoc
 * http://https://github.com/andersondomingues/orca-tools
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
#include "DataConvertionHelper.hpp"

#include <iomanip>

// string hex to int
int strhti(char* buffer, int length) {
    char tmp[length];

    for (int i = 0; i < length; i++)
        tmp[i] = '\0';

    for (int i = 0; i < length; i++) {
        // is [0-9] digit
        if (buffer[i] >= 48 && buffer[i] <= 57) {
            tmp[i] = buffer[i];
        // is [a-f] digit
        } else if (buffer[i] >= 97 && buffer[i] <= 122) {
            tmp[i] = buffer[i];
        } else {
            break;
        }
    }

    return static_cast<int>(strtol(tmp, NULL, 16));
}

// find first occurrence of a character, returns index
int strfind(char* buffer, char find, int limit) {
    for (int i = 0; i < limit; i++)
        if (buffer[i] == find) return i;

    return -1;
}

// byte array to hexa
void hexstr(char* output, char* input, uint32_t integers) {
    uint32_t mask = 0xFFFFFFFF;

    uint32_t* input_i = (uint32_t*)input;
    uint32_t output_i = 0;

    // converts integer by integer
    for (uint32_t i =0; i < integers; i++) {
        // TODO(a): check whether the two lines below are equivalent
        // snprintf(&output[output_i],
        // sizeof(&output[output_i]), "%08x", endswap(input_i[i] & mask));
        sprintf(&output[output_i], "%08x", endswap(input_i[i] & mask));	
        output_i += 8;
    }
}

uint32_t endswap(uint32_t value) {
    uint32_t tmp;

    tmp = ((value << 8) & 0xFF00FF00) | ((value >> 8) & 0xFF00FF);
    value = (tmp << 16) | (tmp >> 16);

    return value;
}
