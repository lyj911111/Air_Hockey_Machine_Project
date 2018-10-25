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
 * from 범위의 value값을 to범위의 mapping값으로 반환.
 */
int Math_Mapping(int val, int from_min, int from_max, int to_min, int to_max);

/*
 * mapping된 값을 역순으로  mapping된 값으로 찾아줌.
 */
int Reverse_Mapping(int map_val, int max, int min);

/*
 * x y 값에 따라 세 범위로 나눠 boundary(경계)값을 정해 주면
 * 	각각의 범위에 따른 명령을 수행.
 */
void Mapped_Value_Controller(int value1, int value2, int low_boundary, int high_boundary);

#endif /* KHY_H_ */
