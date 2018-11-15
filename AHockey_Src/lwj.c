/*
 * lwj.c
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#include "../AHockey_Inc/lwj.h"

#include "../AHockey_Inc/khy.h"

extern TIM_HandleTypeDef htim3;
int cnt = 0;


void Step_Motor_Auto(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Pulse_Limit)
{

	switch(Motor_Select)		//	모터 선택 (왼쪽 또는 오른쪽)
	{
	case MOTOR_LEFT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, Direction);	//	시계, 시계반대 방향 제어.


		break;

	case MOTOR_RIGHT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, Direction);

		break;
	}
}



void Step_Generate_Pulse(uint32_t Number)
{
	cnt = Number;
	HAL_TIM_Base_Start_IT(&htim3);
}

//////////////////////////////////////////////////////////////
/*
 *  스텝모터 쉽게 통제하는 함수.
 *  함수 설명 : 좌우상하를 쉽게 움직일 수 있도록 함.
 *
 *	Step_Stop()            : 멈춤.
 *  Step_GoingRight()      : 오른쪽으로 이동.(→)
 *  Step_GoingLeft()       : 왼쪽로 이동.(←)
 *  Step_GoingDown()       : 아래로 이동.(↓)
 *  Step_GoingUp()         : 위로 이동.(↑)
 *  Step_GoingRight_Up()   : 오른쪽 위로 이동.(↗)
 *  Step_GoingLeft_Up()    : 왼쪽위로 이동.(↖)
 *  Step_GoingRight_Down() : 오른쪽 아래로 이동.(↘)
 *  Step_GoingLeft_Down()  : 왼쪽 아래로 이동.(↙)
 *
 */
//////////////////////////////////////////////////////////////

void Step_Stop(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 80, MOVE_OFF);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 80, MOVE_OFF);
}

void Step_GoingRight(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_ON);
}

void Step_GoingLeft(void)
{
	Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 85, MOVE_ON);
}

void Step_GoingDown(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 85, MOVE_ON);
}

void Step_GoingUp(void)
{
	Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_ON);
}

void Step_GoingRight_Up(void)
{
	Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 85, MOVE_OFF);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_ON);
}

void Step_GoingLeft_Up(void)
{
	Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_OFF);
}

void Step_GoingRight_Down(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_OFF);
}

void Step_GoingLeft_Down(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 85, MOVE_OFF);
	Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 85, MOVE_ON);
}

//////////////////////////////////////////////////////////////
/*
 *  스텝모터 제어 함수 받는 인자 4개.
 *  함수 사용법 : [모터선택] / [방향(시계, 시계반대)] / [Pulse주기를 받아 스텝모터 제어] / [0:정지, 1:움직임].
 *	※주의사항, Motor_Speed 최소값: 90 (가장빠른속도), 최대값: 270 (가장 느린 속도)
 *		========================================
 *		왼쪽모터     오른쪽모터     |   모션
 *		========================================
 *		시계방향     시계방향        |  (←)왼쪽 이동
 *		반시계방향  반시계방향     |  (→)오른쪽 이동
 *		시계방향     반시계방향     |  (↑)위로 이동
 *		반시계방향  시계방향        |  (↓)아래로 이동
 *
 *		정지     	 시계방향         |  (↙)왼쪽 아래 이동
 *		정지     	 반시계방향      |  (↗)오른쪽 위 이동
 *		반시계       정지              |  (↘)오른쪽 아래 이동
 *		시계방향    정지              |  (↖)왼쪽 위 이동


 */
//////////////////////////////////////////////////////////////

void Step_Motor_Control(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_Speed, uint8_t Motor_Stop)
{

	switch(Motor_Select)		//	모터 선택 (왼쪽 또는 오른쪽)
	{
	case MOTOR_LEFT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, Direction);	//	시계, 시계반대 방향 제어.
		TIM1->ARR = Motor_Speed;							//	값을  Auto-Reload Register에 전달.
		TIM1->CCR1 = TIM1->ARR/2;							//	듀티비 50%를 유지.
		if(Motor_Stop == MOVE_OFF)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//	MOVE_OFF 입력시 모터 정지.
		}
		break;

	case MOTOR_RIGHT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, Direction);
		TIM1->ARR = Motor_Speed;
		TIM1->CCR2 = TIM1->ARR/2;
		if(Motor_Stop == MOVE_OFF)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
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
