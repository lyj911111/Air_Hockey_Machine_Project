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
 *  �Լ� ���� : x��ǥ�� 0~20 ���� �޾� �ش� ��ǥ�� �̵���Ŵ.
 *
 *
 */
//////////////////////////////////////////////////////////////

uint32_t pulse_num;		//	0~1400
uint32_t x_coord = 10;		//	0~20
extern uint8_t step_flag;

uint8_t Step_Rx_Data(uint32_t x_data )	//	x��ǥ 0~20 ���� �����͸� ����.
{
	const static uint32_t step_move = 70;
	int32_t x_rel = 0;						//	x���� �����ǥ
	uint8_t dir = 0;

	if(x_data > 20 || x_data < 0 || step_flag != 1)		//	������ ���� ���� ��, ���ܸ��Ͱ� ������� ���� �� ���� ���.
		return 0;

	x_rel = (x_coord - x_data);			//	������ǥ - �̵���ǥ = �����ǥ
	x_coord = x_data;

	if(x_rel > 0)	//	������ ��������
	{
		dir = CLK_WISE;
	}
	else if(x_rel < 0)	//	������ ����������
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

void Init_Step(void)	//	�ʱ� �������� ���� ������ �о��ش�. (����� �̵��Ͽ� )
{
	step_flag = 1;
	Step_pulse(700, MOTOR_LEFT, CNT_CLK_WISE);
	Step_pulse(700, MOTOR_RIGHT, CNT_CLK_WISE);
	pulse_start();
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
 *  �Է��� �޽��� ��ŭ �����̴� ���ܸ���
 *  �Լ�����: [�޽��� ����]/[���ͼ���]/[���͹���]
 *
 *  	�޽� ������ ����.
 *
 *  �߰��������� ���� �� ���ܱ��� 700�� �޽�.
 *  ó���������� ���������� 1400�� �޽�.
 */
//////////////////////////////////////////////////////////////

uint16_t pulse_count1 = 0;
uint16_t pulse_count2 = 0;
uint8_t step_flag = 1;

uint8_t Step_pulse(uint16_t pulse_num,uint16_t motor_select,uint16_t direction)
{
  if(pulse_count1 == 0 && motor_select == MOTOR_LEFT){			//	���� ���� (����)
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, direction);			//	���� ���� ����
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);					//	�޽����� �߻��ϴ� ��Ʈ (Ÿ�̸� ī���� 1 �̿�, main�� __weak�Լ� Ÿ�̸� ���ͷ�Ʈ)
    pulse_count1 = pulse_num * 2;								//	1 �ֱ�� 1����  �޽��� ����� ����.
  }
  else if(pulse_count2 == 0 && motor_select == MOTOR_RIGHT){
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, direction);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
    pulse_count2 = pulse_num * 2;
  }
  else
    return 1;	// �Ҿ��� ����.
  return 0;		// ���������� �Լ��� �Ϸ�� ��
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
 *  Step���� ���� ���� (�޽� �׽�Ʈ ��)
 *  �Լ� ���� : [��] ��ư�� [��] ��ư�� �̿��ؼ� x��ǥ�� �������� ������.
 *
 *	dir : �¿� ������ ������.
 *	temp : �ּ� ������ ���� ����.
 */
//////////////////////////////////////////////////////////////

int dir = 0;	//	���� ����.
int temp = 70;	//	�޽��� ���� (������ ���� ����)
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