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

#define MOVE_OFF		0	//	���� ȸ�� ����
#define	MOVE_ON			1	//	���� ȸ�� ����

void Step_GoingRight(void);
void Step_GoingLeft(void);
void Step_GoingDown(void);
void Step_GoingUp(void);
void Step_GoingRight_Up(void);
void Step_GoingLeft_Up(void);
void Step_GoingRight_Down(void);
void Step_GoingLeft_Down(void);

void Step_Motor_Control(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_Speed, uint8_t Motor_Stop);
void Step_Generate_Pulse(uint32_t Number);

#endif /* LWJ_H_ */
