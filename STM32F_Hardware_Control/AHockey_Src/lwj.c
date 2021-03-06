/*
 * lwj.c
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#include "../AHockey_Inc/lwj.h"
#include "../AHockey_Inc/khy.h"
#include <stdlib.h>


extern TIM_HandleTypeDef htim1;
int cnt = 0;

//////////////////////////////////////////////////////////////
/*
 *  함수 설명 : x좌표값 0~20 값을 받아 해당 좌표로 이동시킴.
 *
 *
 */
//////////////////////////////////////////////////////////////

uint32_t pulse_num;		//	0~1400
uint32_t x_coord = 10;		//	0~20
extern uint8_t step_flag;

uint8_t Step_Rx_Data(uint32_t x_data )	//	x좌표 0~20 값의 데이터를 받음.
{
	const static uint32_t step_move = 70;
	int32_t x_rel = 0;						//	x값의 상대좌표
	uint8_t dir = 0;

	if(x_data > 20 || x_data < 0 || step_flag != 1)		//	비정상 값이 나올 시, 스텝모터가 실행되지 않을 때 실행 취소.
		return 0;

	x_rel = (x_coord - x_data);			//	현재좌표 - 이동좌표 = 상대좌표
	x_coord = x_data;

	if(x_rel > 0)	//	방향을 왼쪽으로
	{
		dir = CLK_WISE;
	}
	else if(x_rel < 0)	//	방향을 오른쪽으로
	{
		dir = CNT_CLK_WISE;
	}

	step_flag = 1;
	Step_pulse( abs(x_rel*step_move) , MOTOR_LEFT, dir);
	Step_pulse( abs(x_rel*step_move) , MOTOR_RIGHT, dir);
	pulse_start();

	return 1;
}

extern uint8_t step_flag;

void Init_Step(void)	//	초기 수동으로 왼쪽 끝으로 밀어준다. (가운데로 이동하여 )
{
	step_flag = 1;
	Step_pulse(700, MOTOR_LEFT, CNT_CLK_WISE);
	Step_pulse(700, MOTOR_RIGHT, CNT_CLK_WISE);
	pulse_start();
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
 *  입력한 펄스값 만큼 움직이는 스텝모터
 *  함수사용법: [펄스의 갯수]/[모터선택]/[모터방향]
 *
 *  	펄스 움직임 정보.
 *
 *  중간지점으로 부터 양 끝단까지 700번 펄스.
 *  처음지점부터 끝지점까지 1400번 펄스.
 */
//////////////////////////////////////////////////////////////

uint16_t pulse_count1 = 0;
uint16_t pulse_count2 = 0;
uint8_t step_flag = 1;

uint8_t Step_pulse(uint16_t pulse_num,uint16_t motor_select,uint16_t direction)
{
  if(pulse_count1 == 0 && motor_select == MOTOR_LEFT){			//	모터 선택 (왼쪽)
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, direction);			//	모터 방향 선택
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);					//	펄스파형 발생하는 포트 (타이머 카운터 1 이용, main의 __weak함수 타이머 인터럽트)
    pulse_count1 = pulse_num * 2;								//	1 주기당 1번의  펄스를 만들기 위함.
  }
  else if(pulse_count2 == 0 && motor_select == MOTOR_RIGHT){
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, direction);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
    pulse_count2 = pulse_num * 2;
  }
  else
    return 1;	// 불안정 종료.
  return 0;		// 성공적으로 함수가 완료될 때
}

void pulse_start()
{
  if(step_flag == 1){
    HAL_TIM_Base_Start_IT(&htim1);
    step_flag = 0;
  }
}

//////////////////////////////////////////////////////////////
/*
 *  Step모터 수동 조작 (펄스 테스트 용)
 *  함수 사용법 : [←] 버튼과 [→] 버튼을 이용해서 x좌표를 수동으로 조작함.
 *
 *	dir : 좌우 방향을 선택함.
 *	temp : 최소 움직임 단위 선택.
 */
//////////////////////////////////////////////////////////////

int dir = 0;	//	방향 선택.
int temp = 70;	//	펄스값 선택 (움직임 단위 선택)
extern uint8_t Rx_data[3];

void Step_Manual_Control(void)
{
	  if(step_flag == 1)
	  {
		  if(Rx_data[0] == 27 && Rx_data[1] == 91)
		  {
			  if(Rx_data[2] == 68)	//	left
			  {
				  dir = CLK_WISE;
			  }
			  else if(Rx_data[2] == 67)	//	Right
			  {
				  dir = CNT_CLK_WISE;
			  }
			  Step_pulse(temp, MOTOR_LEFT, dir);
			  Step_pulse(temp, MOTOR_RIGHT, dir);
			  pulse_start();
		  }

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
