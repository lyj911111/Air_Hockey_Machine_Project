/*
 * khy.h
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#ifndef KHY_H_
#define KHY_H_

#include "stm32f4xx_hal.h"

/*
 * from_min�� from_max ������ value��  from_min�� from_max ������ ������ �����Ͽ� ����
 * */
int math_Map(int val, int from_min, int from_max, int to_min, int to_max);
int reverse_map(int map_val, int max, int min);

#endif /* KHY_H_ */
