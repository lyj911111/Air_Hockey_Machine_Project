/*
 * lwj.c
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#include "lwj.h"

//////////////////////////////////////////////////////////////
/*
 *  스텝모터 제어 함수
 *  함수 사용법 : 모터선택, 방향, PWM신호를 받아 스텝모터 제어
 *
 *
 */
//////////////////////////////////////////////////////////////

void Step_Motor_Control(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_Speed, uint8_t Motor_Stop)
{

	switch(Motor_Select)
	{
	case MOTOR_LEFT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, Direction);	//	시계, 시계반대 방향 제어.
		TIM1->ARR = Motor_Speed;							//	값을  Auto-Reload Register에 전달.
		TIM1->CCR1 = TIM1->ARR/2;							//	듀티비 50%를 유지.
		if(Motor_Stop == NO_MOVE)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		}
		else
		{
			//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
		}
		break;

	case MOTOR_RIGHT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, Direction);
		TIM1->ARR = Motor_Speed;
		TIM1->CCR2 = TIM1->ARR/2;
		if(Motor_Stop == NO_MOVE)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		}
		else
		{
			//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);
		}
		break;
	}
}

//////////////////////////////////////////////////////////////
/*
 *  WatchDog 함수.
 *  함수 사용법 : 0.5초이상 응답이 없을 시, 시스템을 재부팅하는 함수.
 *            WatchDog이 필요한 곳에 함수를 넣으면 된다.
 *            오작동으로 인한 시스템 재부팅시 보드의 빨간색 LED에 불이 들어온다. (즉, Watchdog에 의한 재부팅)
 */
//////////////////////////////////////////////////////////////

void Watch_Dog(void)
{
	//__HAL_IWDG_RELOAD_COUNTER(&hiwdg);   //  System이 Reset되지 않도록 지속적인 카운트한다. (0.5초이상되면 리셋)

	// 시스템이 재부팅 되었는지 확인하는 함수.
	if(RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // 빨간색 LED로 재부팅 여부 확인.
		__HAL_RCC_CLEAR_RESET_FLAGS();
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	}
}
