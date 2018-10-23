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
 * from_min과 from_max 사이의 value를  from_min과 from_max 사이의 값으로 맵핑하여 리턴
 * */
uint32_t math_Map(uint32_t val, uint32_t from_min, uint32_t from_max, uint32_t to_min, uint32_t to_max);


#endif /* KHY_H_ */
