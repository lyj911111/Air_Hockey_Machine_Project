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
 * from ������ value���� to������ mapping������ ��ȯ.
 */
int math_Map(int val, int from_min, int from_max, int to_min, int to_max);

/*
 * mapping�� ���� ��������  mapping�� ������ ã����.
 */
int reverse_map(int map_val, int max, int min);

/*
 * x y ���� ���� �� ������ ���� boundary(���)���� ���� �ָ�
 * 	������ ������ ���� ����� ����.
 */
void map_value_controller(int value1, int value2, int low_boundary, int high_boundary);

#endif /* KHY_H_ */
