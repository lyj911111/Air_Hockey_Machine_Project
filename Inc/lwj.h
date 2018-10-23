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

#define NO_MOVE			0	//	모터 회전 정지

void Step_Motor_Control(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_Speed, uint8_t Motor_Stop);

#endif /* LWJ_H_ */
