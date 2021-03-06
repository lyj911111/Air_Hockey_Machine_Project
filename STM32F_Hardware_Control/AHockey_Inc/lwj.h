/*
 * lwj.h
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#ifndef LWJ_H_
#define LWJ_H_

#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim1;	//	스텝모터 제어 타이머
//IWDG_HandleTypeDef hiwdg;	//	와치독 핸들러

#define MOTOR_LEFT		0	//	왼쪽 모터
#define MOTOR_RIGHT		1	//	오른쪽 모터

#define CLK_WISE		0	//	시계방향 플레그
#define CNT_CLK_WISE	1	//	반시계방향 플레그

#define MOVE_OFF		0	//	모터 회전 정지
#define	MOVE_ON			1	//	모터 회전 가동

void Step_GoingRight(void);
void Step_GoingLeft(void);
void Step_GoingDown(void);
void Step_GoingUp(void);
void Step_GoingRight_Up(void);
void Step_GoingLeft_Up(void);
void Step_GoingRight_Down(void);
void Step_GoingLeft_Down(void);
void Step_Stop(void);

void Step_Motor_Control(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_Speed, uint8_t Motor_Stop);
void Step_Generate_Pulse(uint32_t Number);
void Step_Manual_Control(void);
void Init_Step(void);
uint8_t Step_Rx_Data(uint32_t x_data );

uint8_t Step_pulse(uint16_t pulse_num,uint16_t motor_select,uint16_t direction);
void pulse_start();

#endif /* LWJ_H_ */
