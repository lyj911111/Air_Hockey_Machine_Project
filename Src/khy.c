/*
 * khy.c
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#include "khy.h"

int math_Map(int val, int from_min, int from_max, int to_min, int to_max)
{
	return (val - from_min) * (to_max - to_min)/(from_max - from_min) + to_min;
}

int reverse_map(int map_val, int max, int min)
{
	return (max-map_val)+min;
}
