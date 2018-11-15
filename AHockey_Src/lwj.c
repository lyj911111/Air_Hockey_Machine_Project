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

	switch(Motor_Select)		//	���� ���� (���� �Ǵ� ������)
	{
	case MOTOR_LEFT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, Direction);	//	�ð�, �ð�ݴ� ���� ����.


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
 *  ���ܸ��� ���� �����ϴ� �Լ�.
 *  �Լ� ���� : �¿���ϸ� ���� ������ �� �ֵ��� ��.
 *
 *	Step_Stop()            : ����.
 *  Step_GoingRight()      : ���������� �̵�.(��)
 *  Step_GoingLeft()       : ���ʷ� �̵�.(��)
 *  Step_GoingDown()       : �Ʒ��� �̵�.(��)
 *  Step_GoingUp()         : ���� �̵�.(��)
 *  Step_GoingRight_Up()   : ������ ���� �̵�.(��)
 *  Step_GoingLeft_Up()    : �������� �̵�.(��)
 *  Step_GoingRight_Down() : ������ �Ʒ��� �̵�.(��)
 *  Step_GoingLeft_Down()  : ���� �Ʒ��� �̵�.(��)
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
 *  ���ܸ��� ���� �Լ� �޴� ���� 4��.
 *  �Լ� ���� : [���ͼ���] / [����(�ð�, �ð�ݴ�)] / [Pulse�ֱ⸦ �޾� ���ܸ��� ����] / [0:����, 1:������].
 *	�����ǻ���, Motor_Speed �ּҰ�: 90 (��������ӵ�), �ִ밪: 270 (���� ���� �ӵ�)
 *		========================================
 *		���ʸ���     �����ʸ���     |   ���
 *		========================================
 *		�ð����     �ð����        |  (��)���� �̵�
 *		�ݽð����  �ݽð����     |  (��)������ �̵�
 *		�ð����     �ݽð����     |  (��)���� �̵�
 *		�ݽð����  �ð����        |  (��)�Ʒ��� �̵�
 *
 *		����     	 �ð����         |  (��)���� �Ʒ� �̵�
 *		����     	 �ݽð����      |  (��)������ �� �̵�
 *		�ݽð�       ����              |  (��)������ �Ʒ� �̵�
 *		�ð����    ����              |  (��)���� �� �̵�


 */
//////////////////////////////////////////////////////////////

void Step_Motor_Control(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_Speed, uint8_t Motor_Stop)
{

	switch(Motor_Select)		//	���� ���� (���� �Ǵ� ������)
	{
	case MOTOR_LEFT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, Direction);	//	�ð�, �ð�ݴ� ���� ����.
		TIM1->ARR = Motor_Speed;							//	����  Auto-Reload Register�� ����.
		TIM1->CCR1 = TIM1->ARR/2;							//	��Ƽ�� 50%�� ����.
		if(Motor_Stop == MOVE_OFF)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//	MOVE_OFF �Է½� ���� ����.
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
 *  WatchDog �Լ�.
 *  �Լ� ���� : 0.5���̻� ������ ���� ��, �ý����� ������ϴ� �Լ�.
 *            WatchDog�� �ʿ��� ���� �Լ��� ������ �ȴ�.
 *            ���۵����� ���� �ý��� ����ý� ������ ������ LED�� ���� ���´�. (��, Watchdog�� ���� �����)
 */
//////////////////////////////////////////////////////////////

void Watch_Dog(void)
{
	//__HAL_IWDG_RELOAD_COUNTER(&hiwdg);   //  System�� Reset���� �ʵ��� �������� ī��Ʈ�Ѵ�. (0.5���̻�Ǹ� ����)

	// �ý����� ����� �Ǿ����� Ȯ���ϴ� �Լ�.
	if(RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // ������ LED�� ����� ���� Ȯ��.
		__HAL_RCC_CLEAR_RESET_FLAGS();
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	}
}
