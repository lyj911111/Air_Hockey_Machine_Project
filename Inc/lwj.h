/*
 * lwj.h
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#ifndef LWJ_H_
#define LWJ_H_

#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim1;	//	���ܸ��� ���� Ÿ�̸�
//IWDG_HandleTypeDef hiwdg;	//	��ġ�� �ڵ鷯

#define MOTOR_LEFT		0	//	���� ����
#define MOTOR_RIGHT		1	//	������ ����

#define CLK_WISE		0	//	�ð���� �÷���
#define CNT_CLK_WISE	1	//	�ݽð���� �÷���

#define NO_MOVE			0	//	���� ȸ�� ����

void Step_Motor_Control(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_Speed, uint8_t Motor_Stop);

#endif /* LWJ_H_ */
