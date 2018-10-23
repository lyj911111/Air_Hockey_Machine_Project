/*
 * khy.c
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#include "khy.h"

uint32_t math_Map(uint32_t val, uint32_t from_min, uint32_t from_max, uint32_t to_min, uint32_t to_max)
{
	return (val - from_min) * (to_max - to_min)/(from_max - from_min) + to_min;
}
